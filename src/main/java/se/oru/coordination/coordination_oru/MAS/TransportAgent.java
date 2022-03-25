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


public class TransportAgent extends CommunicationAid{
    protected TrajectoryEnvelopeCoordinatorSimulation tec;
    protected ReedsSheppCarPlanner mp;
    protected Coordinate[] rShape;
    protected Pose startPose;
    protected final int oreCap = 15;

    protected Schedule schedule;

    public ArrayList<Message> missionList = new ArrayList<Message>();


    public TransportAgent(int id){this.robotID = id;}   // for testing

    public TransportAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
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

    public TransportAgent( //old constructor 
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
        TransportAgent This = this;

        This.addRobotToSimulation();

        Thread listenerThread = new Thread() {
            public void run() {
                This.listener();
            }
        };
        listenerThread.start();

        try { Thread.sleep(2000); }
        catch (InterruptedException e) { e.printStackTrace(); }

        Thread stateThread = new Thread() {
            public void run() {
                This.initialState();
            }
        };
        stateThread.start();

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

        this.tec.setFootprint(this.robotID, this.rShape);

        this.tec.placeRobot(this.robotID, this.startPose);

        // Motion planner
        tec.setMotionPlanner(this.robotID, this.mp);

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

            Pose start;
            Pose goal = new Pose(coordinates[0], coordinates[1], coordinates[2]);

            //this.planState(pos); //TODO change in future

            // Setting up the mission
            
            
            if (this.schedule.lastToPose != null) {
                System.out.println("USES LAST TO POSE <------------------------------------------");
                this.mp.setStart(this.schedule.lastToPose);
                start = this.schedule.lastToPose;
            } else {
                this.mp.setStart(this.tec.getRobotReport(this.robotID).getPose());
                start = this.tec.getRobotReport(this.robotID).getPose();
            }
            
            this.mp.setGoals(goal);
            if (!this.mp.plan()) throw new Error ("No path between " + "current_pos" + " and " + goal);
            PoseSteering[] path = this.mp.getPath();
    
            // Create task and add it to schedule
            Task task = new Task(taskID, new Mission(this.robotID, path), 0, m.sender, "NOT STARTED", start, goal, true);
            this.schedule.enqueue(task);

            System.out.println("SCHEDULE SIZE =====" + this.schedule.getSize());

            //this.planState();
        }
    }

    protected Boolean isMissionDone (Task task) {
        // Distance from robots actual position to task goal pose
        return this.tec.getRobotReport(this.robotID).getPose().distanceTo(task.mission.getToPose()) < 0.5;
    }

    /**
     * Function enabling TA to execute the tasks in its schedule. 
     * TODO Add functionality for knowing if robot managed to complete a task or not. 
     */
    protected void executeTasks () {
        while (true) {
            // Execute task while its schedule is not empty
            if (this.schedule.getSize() > 0) {
                Task task = this.schedule.dequeue();
                this.tec.addMissions(task.mission);
                
                while (isMissionDone(task) == false) {
                    try { Thread.sleep(500); }
                    catch (InterruptedException e) { e.printStackTrace(); }
                }

                // if robot managed to complete task 
                String oreChange = task.isSATask ? Integer.toString(this.oreCap) : Integer.toString(-this.oreCap);
                Message doneMessage = new Message(this.robotID, task.taskProvider, "inform", task.taskID + "," + "done" + "," + oreChange);
                this.sendMessage(doneMessage, false);
                this.logTask(task.taskID, "done");

                // if robot didn't manage to complete task
            }
            try { Thread.sleep(500); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
    }


    protected void initialState() {

        while (true){

            // start CNP with DA
            Message bestOffer = this.offerService();
            String[] parts = this.parseMessage(bestOffer, "", true);

            // queue mission to DA
            Pose start;
            if (this.schedule.lastToPose != null) {
                this.mp.setStart(this.schedule.lastToPose);
                start = this.schedule.lastToPose;
            } else {
                this.mp.setStart(this.tec.getRobotReport(this.robotID).getPose());
                start = this.tec.getRobotReport(this.robotID).getPose();
            }

            double[] coordinates = Arrays.stream(parts[2].split(" "))
            .mapToDouble(Double::parseDouble)
            .toArray();
            Pose goal = new Pose(coordinates[0], coordinates[1], coordinates[2]);
            
            this.mp.setGoals(goal);
            if (!this.mp.plan()) throw new Error ("No path between " + "current_pos" + " and " + goal);
            PoseSteering[] path = this.mp.getPath();
    
            // Create task and add it to schedule
            Task task = new Task(Integer.parseInt(parts[0]),
            new Mission(this.robotID, path), 0, bestOffer.sender, "NOT STARTED", start, goal, false);
            this.schedule.enqueue(task);    //TODO fix so it is added after a SA mission


            // wait for SA mission to be added
            while (!this.schedule.isLastTaskSA()){
                try { Thread.sleep(300); }
                catch (InterruptedException e) { e.printStackTrace(); }
            }

        }

    }

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportAgent} calling this
     */
    @Override
    public Message offerService(){

        System.out.println("======================1");

        ArrayList<Integer> receivers = new ArrayList<Integer>(this.robotsInNetwork);
        receivers.removeIf(i -> i<10000);    //draw agents has robotID > 10000
        System.out.println("======================2");

        // broadcast message to all transport agents
        //Pose pos = new Pose(63.0,68.0, 0.0);
        Pose start;
        if (this.schedule.lastToPose != null) {
            start = this.schedule.lastToPose;
        } else {
            start = this.tec.getRobotReport(this.robotID).getPose();
        }
        String startPos = start.getX() + " " + start.getY() + " " + start.getYaw();

        String body = this.robotID + this.separator + startPos;
        Message m = new Message(this.robotID, receivers, "cnp-service", body);
        int taskID = this.sendMessage(m, true);
        System.out.println("======================3");

        //sleep 6 sec before looking at offers
        try { Thread.sleep(2500); }
        catch (InterruptedException e) { e.printStackTrace(); }
        System.out.println("======================4");

        Message bestOffer = this.handleOffers(taskID); //extract best offer
        System.out.println("======================5");

        if (bestOffer != null){        
            // Send response: Mission to best offer sender, and deny all the other ones.
            Message acceptMessage = new Message(robotID, bestOffer.sender, "accept", Integer.toString(taskID) );
            this.sendMessage(acceptMessage);

            receivers.removeIf(i -> i==bestOffer.sender);

            if (receivers.size() > 0){
                // Send decline message to all the others. 
                Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(taskID));
                this.sendMessage(declineMessage);

                //TODO add amout A to be received at time T in schedule
            }
    
            

            return bestOffer;
        }
        return null;
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

                else if (m.type == "offer"){
                    this.offers.add(m);
                }
                
            }
        
            // System.out.println(this.robotID + " -- " + this.robotsInNetwork);
            try { Thread.sleep(1000); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
    }
}
