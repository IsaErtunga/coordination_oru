package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Arrays;
import java.lang.Math;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;


public class TransportAgent extends CommunicationAid{
    //Control parameters
    protected double TIME_WAITING_FOR_OFFERS = 3.0;
    protected String COLOR = "\033[0;32m";

    protected TrajectoryEnvelopeCoordinatorSimulation tec;
    protected ReedsSheppCarPlanner mp;

    protected Coordinate[] rShape;
    protected Pose startPose;
    protected final Double capacity = 15.0;
    protected final int taskCap = 4;

    protected TimeScheduleNew timeSchedule;
    protected long startTime;

    public ArrayList<Message> missionList = new ArrayList<Message>();


    public TransportAgent(int id){this.robotID = id;}   // for testing

    public TransportAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
                        ReedsSheppCarPlanner mp, Pose startPos, Router router){}

    public TransportAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec, ReedsSheppCarPlanner mp, Pose startPos,
                            Router router, long startTime){
            
                            System.out.println("#######################");
                            System.out.println(r_id +" -- constructor");
        
        this.robotID = r_id;
        this.tec = tec;
        this.mp = mp;
        this.startPose = startPos;
        this.startTime = startTime;

        this.timeSchedule = new TimeScheduleNew(startPos, this.capacity, 0.0);

        // enter network and broadcast our id to others.
        router.enterNetwork(this);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);
                
        double xl = 4.0;
	    double yl = 2.8;
        this.rShape = new Coordinate[] {new Coordinate(-xl,yl),new Coordinate(xl,yl),
                                        new Coordinate(xl,-yl),new Coordinate(-xl,-yl)};

    }


    protected double getTime(){
        long diff = System.currentTimeMillis() - this.startTime;
        return (double)(diff)/1000.0;
    }

    protected double getNextTime(){
        double STARTUP_ADD = 10.0;
        synchronized(this.timeSchedule){
            double nextTime = this.timeSchedule.getNextStartTime();
            return (int)(nextTime) == -1 ? this.getTime()+STARTUP_ADD : nextTime;
        }
    }


    /**
     * 
     */
    public void start(){
        TransportAgent This = this;

        This.addRobotToSimulation();

        Thread listenerThread = new Thread() {
            public void run() {
                This.listener();
            }
        };
        listenerThread.start();

        this.sleep(300);

        Thread stateThread = new Thread() {
            public void run() {
                This.initialState();
            }
        };
        stateThread.start();

        this.executeTasks();
    

    }

    /**
     * 
     */
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


    /**
     * Compare robot pose to task end pose to see if Task is finished
     * @param task
     * @return
     */
    protected Boolean isTaskDone (Task task) {
        // Distance from robots actual position to task goal pose
        return this.tec.getRobotReport(this.robotID).getPose().distanceTo(task.toPose) < 0.5;
    }

    /**
     * Function enabling TA to execute the tasks in its schedule. 
     * TODO Add functionality for knowing if robot managed to complete a task or not. 
     */
    protected void executeTasks () {
        while (true) {
            this.sleep(200);

            ArrayList<Task> newEndTimes;
            Task task = null;
            Pose prevTaskFromPose = null;
            
            synchronized(this.timeSchedule){
                prevTaskFromPose = this.timeSchedule.getPoseAtTime(-1.0);
                task = this.timeSchedule.getNextEvent();
            }
            if (task == null) continue;
            this.timeSchedule.printSchedule(this.COLOR);

            // update schedule to actual start of mission and send inform msg to all tasks affected.
            double now = this.getTime();
            double nextStartTime = task.endTime - task.startTime + now;
            synchronized(this.timeSchedule){
                this.timeSchedule.changeOreStateEndTime(task.taskID, nextStartTime);
                newEndTimes = this.timeSchedule.compressSchedule(nextStartTime);
            }
            this.print("starting mission with -->" +task.partner + "\tat time-->"+this.getTime()+"\ttaskStartTime-->"+task.startTime);

            task.startTime = now;
            task.endTime = nextStartTime;
            newEndTimes.add(0, task);
            // start next task
            this.tec.addMissions(this.createMission(task, prevTaskFromPose));

            // send inform msgs informing of new times.
            if ( now - task.startTime > 0.0 ){ //if we start later = send from last to first
                for (int i=newEndTimes.size()-1; i>0; i--){
                    Task t = newEndTimes.get(i);
                    String body = t.taskID+this.separator+"status"+this.separator+t.endTime;
                    this.sendMessage(new Message(this.robotID, t.partner, "inform", body));
                }
            }
            else {  // if we start earlier = send from first to last
                for (int i=0; i<newEndTimes.size(); i++){
                    Task t = newEndTimes.get(i);
                    String body = t.taskID+this.separator+"status"+this.separator+t.endTime;
                    this.sendMessage(new Message(this.robotID, t.partner, "inform", body));
                }
            }
            

            while (!isTaskDone(task)) {
                this.sleep(100);
            }
            Message doneMessage = new Message(this.robotID, task.partner, "inform", task.taskID + this.separator + "done" + "," + task.ore);
            this.sendMessage(doneMessage, false);

        }
    }

    protected void initialState() {
        double oreLevelThreshold = 1.0;
        while (true) {
            ArrayList<Task> abortTasks = new ArrayList<Task>();
            double lastOreState;

            synchronized(this.timeSchedule){
                abortTasks = this.timeSchedule.fixBrokenSchedule();
                lastOreState = this.timeSchedule.getLastOreState();
            }

            for ( Task t : abortTasks ){
                this.sendMessage(new Message(this.robotID, t.partner, "inform", t.taskID+this.separator+"abort"));
            }


            if ( lastOreState <= oreLevelThreshold ){ // book task to get ore
                Message bestOffer = this.offerService(this.getNextTime()); // hold auction with DA's

                if (bestOffer.isNull){ 
                    this.sleep(200);
                    continue;
                }
    
                Task task = this.createTaskFromMessage(bestOffer);

                boolean taskAdded;
                synchronized(this.timeSchedule){ taskAdded = this.timeSchedule.addEvent(task); }

                if ( taskAdded == false ){ // if false then task no longer possible, send abort msg to task partner
                    this.print("TASK ABORTED");
                    this.sendMessage(new Message(this.robotID, task.partner, "inform", Integer.toString(task.taskID)+this.separator+"abort"));
                }

                // this.print("--- schedule ---");
                // this.timeSchedule.printSchedule(this.COLOR);
            }

            else {
                //this.print("WAITING FOR TASK BY STORAGE AGENT");
            }

            this.sleep(200);
        }

    }

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportAgent} calling this
     */
    public Message offerService(double taskStartTime) {

        // Get correct receivers
        ArrayList<Integer> receivers = this.getReceivers(this.robotID, this.robotsInNetwork, "DRAW");

        if (receivers.size() <= 0) return new Message();
        
        this.offers.clear();

        Pose nextPose;
        synchronized(this.timeSchedule){ nextPose = this.timeSchedule.getNextPose(); }
        
        String startPos = this.stringifyPose(nextPose);
        int taskID = this.sendCNPmessage(taskStartTime, startPos, receivers);
    

        double time = this.waitForAllOffersToCome(receivers.size());
    
        Message bestOffer = this.handleOffers(taskID); //extract best offer

        if (!bestOffer.isNull){        
            Message acceptMessage = new Message(robotID, bestOffer.sender, "accept", Integer.toString(taskID) );
            this.sendMessage(acceptMessage);

            receivers.removeIf(i -> i==bestOffer.sender);

            if (receivers.size() > 0){
                Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(taskID));
                this.sendMessage(declineMessage);
            }
        }
        return bestOffer;
    }

    /** This function is locking. will wait for offers from all recipients to arrive OR return on the time delay.
     * 
     * @param nrOfReceivers an int with the number of recipients of the cnp message sent. 
     * @return a double with the time it took before exiting this function.
     */
    protected double waitForAllOffersToCome(int nrOfReceivers){
        double before = this.getTime();

        while ( this.offers.size() < nrOfReceivers && this.getTime() - before <= this.TIME_WAITING_FOR_OFFERS){
            this.sleep(50);
        }
        return (this.getTime() - before);
    }

    /**
     * Helper function for structuring and sending a CNP message. 
     * TODO needs to be changed in various ways. And maybe moved to communicationAid.
     * @param startTime
     * @param receivers
     * @return taskID
     */
    public int sendCNPmessage(double taskStartTime, String startPos, ArrayList<Integer> receivers) {
        // taskID & agentID & pos & startTime 
        String startTimeStr = Double.toString(taskStartTime);
        String body = this.robotID + this.separator + startPos + this.separator + startTimeStr;
        Message m = new Message(this.robotID, receivers, "cnp-service", body);
        return this.sendMessage(m, true);
    }

    /** //TODO looks good for now. will use timeSchedule.evaluateTimeSlot() in future
     * 
     * HandleOffers is called from a SA, to either accept the offer of a TA, or deny it.
     * @return the message that is the best offer
     */
    public Message handleOffers(int taskID) {
        Message bestOffer = new Message();
        int offerVal = 0;
    
        for (Message m : this.offers) {
            // SCHEDULE: Extract startTime & endTime and see if it fits into schedule
            double taskStartTime = Double.parseDouble(parseMessage(m, "startTime")[0]);
            double endTime = Double.parseDouble(parseMessage(m, "endTime")[0]);
            
            boolean taskPossible;
            synchronized(this.timeSchedule){ taskPossible = this.timeSchedule.isTaskPossible(taskID, taskStartTime, endTime); }

            if (taskPossible) {
                String[] mParts = this.parseMessage( m, "", true); // sort out offer not part of current auction(taskID)

                if ( Integer.parseInt(mParts[0]) == taskID ){
                    int val = Integer.parseInt(mParts[1]);

                    if (val > offerVal){
                        offerVal = val;
                        bestOffer = new Message(m);
                    }
                }
            }
        }
        return bestOffer;
    }

    /** handleService is called from within a TA, when a TA did a {@link offerService}
     * @param m the message with the service
     * @param robotID the robotID of this object
     * @return true if we send offer = we expect resp.
     */
    public boolean handleService(Message m) { 
        this.print("handeling cnp at time-->"+ String.format("%.2f",this.getTime()) );

        double availabeOre;
        int scheduleSize;
        synchronized(this.timeSchedule){
            availabeOre = this.timeSchedule.getLastOreState();
            scheduleSize = this.timeSchedule.getSize();
        }

        if ( scheduleSize > this.taskCap ) return false;
        if (availabeOre <= 0.01) return false;   //if we dont have ore dont act 

        Pose pos = this.timeSchedule.getNextPose();

        Task SATask = this.createTaskFromServiceOffer(m, availabeOre, pos);

        boolean taskPossible;
        synchronized(this.timeSchedule){ taskPossible = this.timeSchedule.isTaskPossible(SATask); }
        if ( !taskPossible ) return false;    // task doesnt fit in schedule

        int offerVal = this.calculateOffer(SATask);
        
        if ( offerVal <= 0.01 ) return false;

        synchronized(this.timeSchedule){ this.timeSchedule.addEvent(SATask); }

        // this.print("--- schedule ---");
        // this.timeSchedule.printSchedule(this.COLOR);
        this.print("sending cnp-msg at time-->"+ String.format("%.2f",this.getTime()) );

        this.sendMessage(this.createOfferMsgFromTask(SATask, offerVal, availabeOre));

        return true;
    }

    /** When receiving a cnp-message, this function will create a task from that message.
     * Only used in {@link handleService}.
     * @param m the message from the auctioneer
     * @param ore the amount of ore the task is about
     * @return a Task with attributes extracted from m
     */
    protected Task createTaskFromServiceOffer(Message m, double ore, Pose taskStartPos){
        String[] mParts = this.parseMessage(m, "", true);

        Pose SApos = this.posefyString(mParts[2]);
        // PoseSteering[] path = this.getPath(this.pStorage, this.mp, this.robotID, m.sender, taskStartPos, SApos);
        //PoseSteering[] path = this.calculatePath(this.mp, taskStartPos, SApos);
        // double pathDist = this.calculatePathDist(path);
        // double pathTime = this.calculateDistTime(pathDist);
        double pathDist = taskStartPos.distanceTo(SApos);
        double pathTime = this.calculateDistTime(pathDist) + 5.0;

        //double startTime = Double.parseDouble(mParts[3]);
        double taskStartTime;
        synchronized(this.timeSchedule){ taskStartTime = this.timeSchedule.getNextStartTime(); }
        
        double endTime = taskStartTime + pathTime;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -ore, taskStartTime, endTime, pathTime, taskStartPos, SApos);
    }
    
    /**
     * Used to generate a response message from a task. Called from {@link handleService}
     * after creating a task with {@link createTaskFromServiceOffer}.
     * @param t a Task that is unactive = t.isActive = false
     * @param offer an int that is the calculated evaluation of the service related to t
     * @param ore a double representing the ore amount the task handels
     * @return returns a Message with attributes extracted from the parameters
     */
    protected Message createOfferMsgFromTask(Task t, int offer, double ore){
        String s = this.separator;

        String startPoseStr = this.stringifyPose(t.fromPose);
        String endPoseStr = this.stringifyPose(t.toPose);
        String body = t.taskID +s+ offer +s+ startPoseStr +s+ 
                      endPoseStr +s+ t.startTime +s+ t.endTime +s+ ore;

        return new Message(this.robotID, t.partner, "offer", body);
    }
    
    /**
     * Calculates and returns offer based on distance and ore-level
     *   OLD:   
        // if (t.pathDist <= 2.0) return 0;
        // double dist = 100.0 * 1.0 / t.pathDist;
        // return (int)(dist);
     * @param t
     * @return offer
     */
    protected int calculateOffer(Task t){
        int offer;
        if (t.pathDist > 0.5) {
            double oreLevel = this.timeSchedule.getOreStateAtTime(this.getTime());
            int fullOreBonus = 1000;
            if (oreLevel >= t.ore) {
                offer = this.calcCDF(t.pathDist, 500) + fullOreBonus;
            }
            else {
                offer = this.calcCDF(t.pathDist, 500) + (int)(fullOreBonus * oreLevel/t.ore);
            }
        }
        else {
            offer = 0;
        }
        return offer;
    }   

    /**
     * 
     * @param startPose
     * @param endPose
     * @return
     */
    public Mission createMission(Task task, Pose prevToPose) {
        return new Mission(this.robotID, this.getPath( this.mp, prevToPose, task.toPose ));
    }


    /** this function holds the logic for handeling messages with type 'inform'. 
     * 
     * @param m the message with the 'inform'-type.
     */
    protected void handleInformMessage(Message m){
        int taskID = Integer.parseInt(this.parseMessage(m, "taskID")[0]);
        String informVal = this.parseMessage(m, "informVal")[0];
        
        if (informVal.equals(new String("done"))) {
            synchronized(this.timeSchedule){ this.timeSchedule.removeEvent(taskID); }
        }

        else if (informVal.equals(new String("status"))) {
            double newEndTime = Double.parseDouble(this.parseMessage(m, "", true)[2]);
            Task taskToAbort = null;

            synchronized(this.timeSchedule){
                taskToAbort = this.timeSchedule.updateTaskEndTimeIfPossible(taskID, newEndTime); // this function aborts task from schedule
            }
            if ( taskToAbort != null ){
                this.sendMessage(new Message(this.robotID, taskToAbort.partner, "inform", taskToAbort.taskID+this.separator+"abort"));
                this.print("sending ABORT msg. taskID-->"+taskID+"\twith-->"+m.sender );
            }
            else {this.print("updated without conflict-->"+taskID +"\twith-->"+ m.sender);}

        }

        else if (informVal.equals(new String("abort"))) {
            this.print("got ABORT MSG! taskID-->"+taskID+"\twith-->"+m.sender );
            synchronized(this.timeSchedule){
                this.timeSchedule.abortEvent(taskID);
                
            }
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
        
            synchronized(this.inbox){
                inbox_copy = new ArrayList<Message>(this.inbox);
                this.inbox.clear();
            }

            for (Message m : inbox_copy){
                //TODO fix new msg type 'echo' to respond to 'hello-world'.
                // this will help us remove this.activeTasks
                int taskID = Integer.parseInt(this.parseMessage(m, "taskID")[0]);
                
                if (m.type == "hello-world"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                    this.sendMessage( new Message( m.receiver.get(0), m.sender, "echo", Integer.toString(taskID)));
                }

                else if (m.type == "echo"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                }

                else if (m.type == "accept"){
                    boolean eventAdded;
                    synchronized(this.timeSchedule){ eventAdded = this.timeSchedule.setEventActive(taskID); }

                    if ( eventAdded == false ) {
                        this.sendMessage(new Message(this.robotID, m.sender, "inform", taskID+this.separator+"abort"));
                        this.print("sending ABORT msg. taskID-->"+taskID+"\twith-->"+m.sender );
                    }
                    this.print("---schedule---");
                    this.timeSchedule.printSchedule(this.COLOR);
                }

                else if (m.type == "decline"){
                    synchronized(this.timeSchedule){
                        boolean successfulRemove = this.timeSchedule.removeEvent(taskID);
                        this.print("remove of task -->"+taskID+ "\tsuccess-->"+successfulRemove);
                    }
                    
                }

                else if (m.type == "cnp-service"){
                    this.handleService(m);
                }

                else if (m.type == "offer"){
                    this.offers.add(m);
                }

                else if (m.type.equals(new String("inform"))) {
                    this.handleInformMessage(m);                    
                }
                
            }
            this.sleep(100);
        }
    }

    /**simlpe print func with color and robotID included
     * @param s string to be printed
     */
    protected void print(String s){
        System.out.println(this.COLOR+this.robotID+"\t" + s + "\033[0m");
    }

}