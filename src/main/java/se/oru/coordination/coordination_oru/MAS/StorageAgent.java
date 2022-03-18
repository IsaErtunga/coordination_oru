package se.oru.coordination.coordination_oru.MAS;

/*
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;

import org.sat4j.ExitCode;


import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
*/
import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.MAS.Router;


public class StorageAgent extends CommunicationAid{
    protected Pose startPose;
    protected double capacity;  // capacity of storage = max ore it can store in TONS
    protected double amount;    // the current amount it has stored in TONS

    public boolean beingUsed = false;

    public StorageAgent(int id){this.robotID = id;}     // for testing

    public StorageAgent(int r_id, Router router, double capacity, Pose startPos ){
        System.out.println("====storage contrsuctor=====");
        this.robotID = r_id;
        this.capacity = capacity;
        this.amount = 0;
        this.startPose = startPos;

        router.enterNetwork(this);

        String type = "hello-world";
        this.sendMessage(new Message(this.robotID, type, ""), true);

    }

    public void start(){
        StorageAgent This = this;
        Thread listener = new Thread() {
            public void run() {
                This.listener();
            }
        };
        listener.start();

        try { Thread.sleep(2000); }
        catch (InterruptedException e) { e.printStackTrace(); }

        this.offerService();

    }

    public void addOre(double ore){
        
        if (this.amount + ore <= this.capacity && ore>0) this.amount = this.amount + ore;
    }

    public double dumpOre(double reqAmount){
        double ret = 0.0;
        
        if (reqAmount > 0){
            if (this.amount - reqAmount < 0){
                ret = this.amount;
                this.amount = 0.0;
            }
            else{
                ret = reqAmount;
                this.amount = this.amount - reqAmount;
            } 
        }
        return ret;
    }

    public void communicateState(){
        // coms

    }

    public void taskHandler(int taskID, Message m){
        String[] taskInfo = this.activeTasks.get(taskID).split(this.separator);

        if (taskInfo[0] == "hello-world" && !this.robotsInNetwork.contains(m.sender)){
            this.robotsInNetwork.add(m.sender);
        }

    }

    public void listener(){
        ArrayList<Message> inbox_copy;

        while(true){
        
            synchronized(inbox){
                inbox_copy = new ArrayList<Message>(this.inbox);
                this.inbox.clear();
            }

            for (Message m : inbox_copy){
                if (m.type == "hello-world"){
                    this.robotsInNetwork.add(m.sender);
                    this.sendMessage(
                        new Message( m.receiver.get(0), m.sender, "accept", m.body));
                } 

                else if (m.type == "offer"){
                    this.offers.add(m);
                }

                else if (m.type == "accept"){
                    this.taskHandler(Integer.parseInt(m.body), m);
                }

                
                else if (m.type == "inform") {
                    // TA informs SA when its done with a task.
                    String message = this.parseMessage(m, "informVal")[0]; 
                    if (message == "abort") {
                        // Create a new task. 
                        offerService();
                    } 
                    else if (message == "done") {
                        
                    }
                    else if (message == "result") {
                        
                    }
                    
                }

                else if (m.type == "cnp-service"){
                    this.handleService(m);
                }
                
            }

            System.out.println(this.robotID + " -- " + this.robotsInNetwork);
            try { Thread.sleep(1000); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }

    }

}
