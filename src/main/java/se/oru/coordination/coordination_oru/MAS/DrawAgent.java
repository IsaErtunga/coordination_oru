package se.oru.coordination.coordination_oru.MAS;
/*
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;

import org.sat4j.ExitCode;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.MAS.Router;
*/
import java.util.ArrayList;
import java.util.Arrays;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.MAS.Router;
import se.oru.coordination.coordination_oru.MAS.Schedule;

import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.Mission;

public class DrawAgent extends CommunicationAid{

    private final static double finalXPos = 4.0;
    protected double initalXPos;

    protected Pose pos;
    protected double amount;
    protected double capacity; 
    protected Schedule schedule;

    public DrawAgent(int robotID, Router router, double capacity, Pose pos){
        this.robotID = robotID; // drawID >10'000
        this.capacity = capacity;
        this.amount = capacity; // 100% full in beginning
        this.pos = pos;
        this.initalXPos = pos.getX();

        this.schedule = new Schedule();

        router.enterNetwork(this.robotID, this.inbox, this.outbox);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);

    }

    public void takeOre(double a){
        // alter ore amount
        if (this.amount - a > 0.0) this.amount -= a;
        else this.amount = 0;

        // alter position
        /*
        100% left -> 4+36 = x.pos = 40
        0% left -> 4+0 = x.pos = 4
        */
        double x = this.finalXPos + (this.initalXPos - this.finalXPos) * this.amount / this.capacity;
        this.pos = new Pose( x, pos.getY(), pos.getYaw() );

    }

    public void taskHandler(int taskID, Message m){
        String[] taskInfo = this.activeTasks.get(taskID).split(this.separator);

        if (taskInfo[0] == "hello-world" && !this.robotsInNetwork.contains(m.sender)){
            this.robotsInNetwork.add(m.sender);
        }

        else if(taskInfo[0].equals("offer")){   // we sent an offer to a SA and got accept reply
            System.out.println(this.robotID + ", in taskhandler: " + taskInfo);
            System.out.println(Arrays.toString(taskInfo));
            //TODO does this agent do something? I think it does nothing.

            // log pick up in schedule.
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

                else if (m.type == "accept"){
                    this.taskHandler(Integer.parseInt(m.body), m);
                }

                else if (m.type == "decline"){
                    //TODO add case
                }

                else if (m.type == "cnp-service"){
                    this.handleService(m);
                }
                
            }

            try { Thread.sleep(1000); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }

    }

    @Override
    public boolean handleService(Message m){ 
        if (m.type != "cnp-service") return false;

        String[] mParts = this.parseMessage( m, "", true);

        // get pose of TA
        double[] coordinates = Arrays.stream(mParts[2].split(" "))
            .mapToDouble(Double::parseDouble)
            .toArray();

        //calc euclidean dist between DA -> TA, and capacity evaluation
        //TODO also include schedule: look if other agent will collect ore here at same time.
        //TODO add poseSteering.length

        double dist_eval = this.pos.distanceTo(new Pose(coordinates[0], coordinates[1], coordinates[2]));
        double capacity_eval = 100.0 * this.amount / this.capacity; 
        
        // generate offer..
        double offer = dist_eval + capacity_eval;
        String body = mParts[0] + this.separator + offer;
        Message resp = new Message(this.robotID, m.sender, "offer", body);
    
        //send offer and log event
        this.sendMessage(resp);
        this.logTask(Integer.parseInt(mParts[0]),
            "offer" + this.separator + m.sender + this.separator + mParts[2] );
        
        //System.out.println(this.robotID + ", task: " + this.activeTasks.get(Integer.parseInt(mParts[0])));
        return true;

    }
    
}
