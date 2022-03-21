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
import java.util.Arrays;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;


public class RobotAgent extends CommunicationAid{
    protected TrajectoryEnvelopeCoordinatorSimulation tec;
    protected ReedsSheppCarPlanner mp;
    protected Coordinate[] rShape;
    protected Pose startPose;

    protected Schedule schedule;

    public ArrayList<Message> missionList = new ArrayList<Message>();



    public RobotAgent(int id){this.robotID = id;}   // for testing

    public RobotAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
                        ReedsSheppCarPlanner mp, Pose startPos, Router router ){
            
                            System.out.println("#######################");
                            System.out.println(r_id +" -- constructor");
        
        this.robotID = r_id;
        this.tec = tec;
        this.mp = mp;
        this.startPose = startPos;

        this.schedule = new Schedule();

        // enter network and broadcast our id to others.
        router.enterNetwork(this);

        String type = "hello-world";
        this.sendMessage(new Message(this.robotID, type, ""), true);
                
        double xl = 5.0;
	    double yl = 3.7;
        this.rShape = new Coordinate[] {new Coordinate(-xl,yl),new Coordinate(xl,yl),
                                        new Coordinate(xl,-yl),new Coordinate(-xl,-yl)};

    }

    public RobotAgent( //old constructor 
        int r_id,
        TrajectoryEnvelopeCoordinatorSimulation tec,
        ReedsSheppCarPlanner mp,
        Coordinate[] shape ){
        
        this.robotID = r_id;
        this.tec = tec;
        this.mp = mp;
        this.rShape = shape;
    }


    public void start(){
        RobotAgent This = this;

        This.addRobotToSimulation();
        System.out.println("----------------THREAD NOT STARTED ----------------");
        Thread listenerThread = new Thread() {
            public void run() {
                This.listener();
            }
        };
        listenerThread.start();
        System.out.println("----------------THREAD STARTED ----------------");
        this.executeTasks();

        //try { Thread.sleep(5000); }
        //catch (InterruptedException e) { e.printStackTrace(); }

    }

    public void addRobotToSimulation(){
        // add robot to simulation prep...
        double MAX_ACCEL = 10.0;
	    double MAX_VEL = 20.0;

        this.tec.setForwardModel(this.robotID, new ConstantAccelerationForwardModel(
            MAX_ACCEL, 
            MAX_VEL, 
            this.tec.getTemporalResolution(), 
            this.tec.getControlPeriod(), 
            this.tec.getRobotTrackingPeriodInMillis(this.robotID)));

        tec.setFootprint(this.robotID, this.rShape);

        tec.placeRobot(this.robotID, this.startPose);

    }

    public void communicateState(int i){
        this.planState();
    }

    public void planState(){
        Task task = this.schedule.dequeue();
        this.tec.addMissions(task.mission);
    }

    /**
     * activeTasks is a datastructure storing tasks with their id for a robot. 
     * This function checks if the taskID of a certain message is contained in the active tasks.
     * If: 
     * Task-type is "hello-world", the robot should add the accepting robot to its network. 
     * @param taskID
     * @param m
     */
    public void taskHandler(int taskID, Message m){
        
        String[] taskInfo = this.activeTasks.get(taskID).split(this.separator);
        
        if (taskInfo[0] == "hello-world" && !this.robotsInNetwork.contains(m.sender)){
            this.robotsInNetwork.add(m.sender);
        }
        
        else if(taskInfo[0].equals("offer")){   // we sent an offer to a SA and got accept reply
            System.out.println(this.robotID + ", in taskhandler: " + taskInfo);
            System.out.println(Arrays.toString(taskInfo));
            //TODO do mission
            //String[] mParts = this.parseMessage( m, "", true);

            double[] coordinates = Arrays.stream(taskInfo[2].split(" "))
            .mapToDouble(Double::parseDouble)
            .toArray();

            Pose goal = new Pose(coordinates[0], coordinates[1], coordinates[2]);

            //this.planState(pos); //TODO change in future

            // Setting up the mission
            
            if (this.schedule.lastToPose != null) {
                this.mp.setStart(this.schedule.lastToPose);
            } else {
                this.mp.setStart(this.tec.getRobotReport(this.robotID).getPose());
            }
            
            this.mp.setGoals(goal);
            if (!this.mp.plan()) throw new Error ("No path between " + "current_pos" + " and " + goal);
            PoseSteering[] path = this.mp.getPath();
    
            // Create task and add it to schedule
            Task task = new Task(taskID, new Mission(this.robotID, path), 0, m.sender, "NOT STARTED");
            this.schedule.enqueue(task);

            System.out.println("SCHEDULE SIZE =====" + this.schedule.getSize());

            //this.planState();
        }
    }

    protected Boolean isMissionDone (Task task) {
        // Distance from robots actual position to task goal pose
        System.out.println("______________________________________________________________________________________________");
        System.out.println("ACTUAL POSITION: " + this.tec.getRobotReport(this.robotID).getPose().toString());
        System.out.println("GOAL : " + task.mission.getToPose().toString());

        // System.out.println("DISTANCE: ---> " + this.tec.getRobotReport(this.robotID).getPose().distanceTo(task.mission.getToPose()));
        System.out.println("______________________________________________________________________________________________");
        if (this.tec.getRobotReport(this.robotID).getPose().distanceTo(task.mission.getToPose()) < 0.5) {
            System.out.println(":::::::::::::::::::MISSION DONE, DISTANCE TO CLOSE::::::::::::ROBOT: "+ this.robotID);
            return true;
        }
        return false;
    }

    protected void executeTasks () {
        while (true) {
            if (this.schedule.getSize() > 0) {
                Task task = this.schedule.dequeue();
                this.tec.addMissions(task.mission);
                
                while (isMissionDone(task) == false) {
                    System.out.println("PPPPPPPPPPPPPPPP -- HÄR");
                    try { Thread.sleep(500); }
                    catch (InterruptedException e) { e.printStackTrace(); }
                }
            }
            try { Thread.sleep(500); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
    }


    /**
     * Periodically checks the inbox of a robot.
     * Reacts to different messages depending on their message type.
     * -------------------------------------------------------------
     * if type == hello-world: add sending robot to its network
     * if type == res-offer: 
     * if type == accept: see taskHandler function. 
     */
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
                    //remove task from activeTasks
                }

                else if (m.type == "cnp-service"){
                    this.handleService(m);
                }
                
            }
        
            // System.out.println(this.robotID + " -- " + this.robotsInNetwork);
            try { Thread.sleep(1000); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
    }
}


/* OLD */

/*

public void taskHandler(int taskID, Message m){
        String[] taskInfo = this.activeTasks.get(taskID).split(this.separator);

        if (taskInfo[0] == "hello-world" && !this.robotsInNetwork.contains(m.sender)){
            this.robotsInNetwork.add(m.sender);
        }

        else if(taskInfo[0] == "offer"){   // we sent an offer to a SA and got accept reply
            //TODO do mission
            //String[] mParts = this.parseMessage( m, "", true);

            double[] coordinates = Arrays.stream(taskInfo[3].split(this.separator))
            .mapToDouble(Double::parseDouble)
            .toArray();

            Pose pos = new Pose(coordinates[0], coordinates[1], coordinates[2]);

            this.planState(pos); //TODO change in future

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
*/