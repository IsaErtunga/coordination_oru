package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.Arrays;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;


public class TransportTruckAgent extends CommunicationAid{
    //Control parameters
    protected double TIME_WAITING_FOR_OFFERS = 3.0;

    protected TrajectoryEnvelopeCoordinatorSimulation tec;
    protected ReedsSheppCarPlanner mp;

    protected Coordinate[] rShape;
    protected Pose startPose;
    protected final Double capacity = 15.0;
    protected Double holdingOre = 0.0;
    protected final int taskCap = 4;

    // Begins at 4. Will iterate through SW, NW, NE, SE
    protected int cornerState = 4;

    protected TimeScheduleNew timeSchedule;
    protected long startTime;

    public ArrayList<Message> missionList = new ArrayList<Message>();


    public TransportTruckAgent(int id){this.robotID = id;}   // for testing

    public TransportTruckAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
                        ReedsSheppCarPlanner mp, Pose startPos, Router router){}

    public TransportTruckAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
                        ReedsSheppCarPlanner mp, Pose startPos, Router router, long startTime){
            
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
                
        double xl = 5.0;
	    double yl = 3.7;
        this.rShape = new Coordinate[] {new Coordinate(-xl,yl),new Coordinate(xl,yl),
                                        new Coordinate(xl,-yl),new Coordinate(-xl,-yl)};

    }


    protected double getTime(){
        //System.out.println(this.robotID+"\ttime---> "+(System.currentTimeMillis() - this.startTime));
        long diff = System.currentTimeMillis() - this.startTime;
        return (double)(diff)/1000.0;
    }

    protected double getNextTime(){
        double STARTUP_ADD = 5.0;
        double nextTime = this.timeSchedule.getNextStartTime();
        return nextTime == -1.0 ? this.getTime()+STARTUP_ADD : nextTime;
    }

    /**
     * Updates which corner the TTA is on.
     */
    protected int incrementCornerState() {
        if (this.cornerState == 4) {
            this.cornerState = 1;
        } 
        else {
            this.cornerState = this.cornerState + 1;
        }
        System.out.println("CORNER STATE: "+ this.cornerState);
        return this.cornerState;
    }


    /**
     * 
     */
    public void start(){
        TransportTruckAgent This = this;

        This.addRobotToSimulation();

        Thread listenerThread = new Thread() {
            public void run() {
                This.listener();
            }
        };
        listenerThread.start();

        this.sleep(500);

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
            int size;
            synchronized(this.timeSchedule){size = this.timeSchedule.getSize(); }
            // Execute task while its schedule is not empty
            if (size > 0) {
                Task task = null;
                synchronized(this.timeSchedule){ task = this.timeSchedule.getNextEvent(); }
                if (task == null) {
                    continue;
                }

                this.tec.addMissions(this.createMission(task));
 
                while (!isTaskDone(task)) {
                    this.sleep(100);
                }
                if (task.partner != -1){
                    Message doneMessage = new Message(this.robotID, task.partner, "inform", task.taskID + this.separator + "done" + this.separator + task.ore);
                    this.sendMessage(doneMessage, false);
                }

                // if robot didn't manage to complete task
            }
            this.sleep(500);
        }
    }

    protected void initialState() {
        double oreLevelThreshold = 1.0;
        while (true) {
            double lastOreState;
            synchronized(this.timeSchedule){
                lastOreState = this.timeSchedule.getLastOreState();
            }
            
            if (lastOreState <= oreLevelThreshold) { 
                // book task to get ore
                Message bestOffer = this.offerService(this.getNextTime());

                if (bestOffer.isNull){ 
                    this.sleep(200);
                    continue;
                }
    
                Task task = this.createTaskFromOfferMessage(bestOffer);

                boolean taskAdded;
                synchronized(this.timeSchedule){ taskAdded = this.timeSchedule.addEvent(task); }

                if ( taskAdded != true ){ // if false then task no longer possible, send abort msg to task partner
                    this.print("TASK ABORTED");
                    this.sendMessage(new Message(this.robotID, task.partner, "inform", Integer.toString(task.taskID)+this.separator+"abort"));
                }
    
            }

            else {
                // Deliver ore 
                Task deliverTask = createDeliverTask();
                boolean taskAdded;
                synchronized(this.timeSchedule){ taskAdded = this.timeSchedule.addEvent(deliverTask); }

                if ( taskAdded != true ){ // if false then task no longer possible, send abort msg to task partner
                    this.print("TASK ABORTED");
                    this.sendMessage(new Message(this.robotID, deliverTask.partner, "inform", deliverTask.taskID+this.separator+"abort"));
                }
                // this.timeSchedule.printSchedule("");
            }

            this.sleep(500);
        }

    }

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportTruckAgent} calling this
     */
    public Message offerService(double taskStartTime) {

        // Get correct receivers
        ArrayList<Integer> receivers = this.getReceivers(this.robotID, this.robotsInNetwork, "STORAGE");

        if (receivers.size() <= 0) return new Message();
        
        this.offers.clear();
        
        Pose nextPose;
        synchronized(this.timeSchedule){ nextPose = this.timeSchedule.getNextPose(); }

        String startPos = this.stringifyPose(nextPose);
        int taskID = this.sendCNPmessage(taskStartTime, startPos, receivers);
    

        double time = this.waitForAllOffersToCome(receivers.size());
        this.print("time waited for offers-->"+time);
    
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
        int SAOrderVAl = 0;
    
        for (Message m : this.offers) {
            // SCHEDULE: Extract startTime & endTime and see if it fits into schedule
            double taskStartTime = Double.parseDouble(parseMessage(m, "startTime")[0]);
            double endTime = Double.parseDouble(parseMessage(m, "endTime")[0]);

            boolean taskPossible;
            synchronized(this.timeSchedule){ taskPossible = this.timeSchedule.isTaskPossible(taskID, taskStartTime, endTime); }
  
            if(taskPossible) {
                String[] mParts = this.parseMessage( m, "", true); // sort out offer not part of current auction(taskID)

                if ( Integer.parseInt(mParts[0]) == taskID ){
                    int val = Integer.parseInt(mParts[1]);
                    int SAOrderNumber = m.sender - 5000;

                    if (val > offerVal) {
                        if (val < offerVal * 1.1) {
                            // If only 10% larger or less
                            if (SAOrderNumber > SAOrderVAl) {
                                // Check precedence
                                offerVal = val;
                                SAOrderVAl = SAOrderNumber;
                                bestOffer = new Message(m);
                            }
                        }
                        else {
                            offerVal = val;
                            SAOrderVAl = SAOrderNumber;
                            bestOffer = new Message(m);
                        }
                    }
                    else {
                        if (val > offerVal * 0.9) {
                            // If only 10% smaller or less
                            if (SAOrderNumber > SAOrderVAl) {
                                // Check precedence
                                offerVal = val;
                                SAOrderVAl = SAOrderNumber;
                                bestOffer = new Message(m);
                            }
                        }
                    }
                }
            }
        }
        return bestOffer;
    }


    /**
     * 
     * @param message
     * @return Task
     */
    protected Task createTaskFromOfferMessage(Message m) {
        String[] mParts = this.parseMessage(m, "", true);
        // replace intexes
        
        // Task(int taskID, int partner, boolean isActive, double ore, double startTime, double endTime, double dist, Pose fromPose, Pose toPose) {
        return new Task(Integer.parseInt(mParts[0]), m.sender, true, this.capacity, Double.parseDouble(mParts[4]),
                        Double.parseDouble(mParts[5]), this.posefyString(mParts[2]), this.posefyString(mParts[3]));
    }

    /**
     * Function that creates a task for the TTA to deliver ore out of mine. 
     * @return Task deliverTask
     */
    protected Task createDeliverTask() {
        Pose robotPos;
        synchronized(this.timeSchedule) {
            robotPos = this.timeSchedule.getNextPose();
        }
        Pose deliveryPos = new Pose(245.0, 105.0, Math.PI);	
        PoseSteering[] path = this.calculatePath(this.mp, robotPos, deliveryPos);
        double pathDist = this.calculatePathDist(path);
        double pathTime = this.calculateDistTime(pathDist);
        double taskStartTime = this.getNextTime();
        double endTime = taskStartTime + pathTime;
        Task deliverTask = new Task(this.tID(), -1, true, -this.capacity, taskStartTime, endTime, endTime, robotPos, deliveryPos);
        return deliverTask;
    }

    /**
     * 
     * @param startPose
     * @param endPose
     * @return
     */
    public Mission createMission(Task task) {
        this.timeSchedule.printSchedule("");
        this.mp.setStart(task.fromPose);
        //Pose[] goals = {task.NE, task.SE, task.SW, task.toPose};
        // if ore is positive, it means that it will fetch ore
        this.mp.setGoals(this.navigateCorrectly(task, task.ore > 0.0));
        if (!this.mp.plan()) throw new Error ("No path between " + "current_pos" + " and " + task.toPose);
        PoseSteering[] path = this.mp.getPath();
        return new Mission(this.robotID, path);
    }

    /**
     * 
     * @param task
     * @param toSA
     * @return
     */
    public Pose[] navigateCorrectly(Task task, boolean toSA) {
        ArrayList<Pose> corners = new ArrayList<Pose>();
        int lastCorner;
        if (toSA) {
            // IF PICKUP ORE
            lastCorner = 1;
        } else {
            lastCorner = 3;
        }

        while (this.cornerState != lastCorner) {
            corners.add(task.corners[this.incrementCornerState()-1]);
        }
        corners.add(task.toPose);
        
        return corners.toArray(new Pose[0]);
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
                //TODO fix new msg type 'echo' to respond to 'hello-world'.
                // this will help us remove this.activeTasks
                int taskID = Integer.parseInt(this.parseMessage(m, "taskID")[0]);
                
                if (m.type == "hello-world"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                    this.sendMessage( new Message( m.receiver.get(0), m.sender, "echo", Integer.toString(taskID)));
                }

                if (m.type == "echo"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                }

                else if (m.type == "accept"){
                    // Add or abort (send message)
                    boolean eventAdded;
                    synchronized(this.timeSchedule){ eventAdded = this.timeSchedule.setEventActive(taskID); }

                    if ( eventAdded == false ) {
                        this.sendMessage(new Message(this.robotID, m.sender, "inform", taskID+this.separator+"abort"));
                    }
                }

                else if (m.type == "decline"){
                    //remove task from activeTasks
                    synchronized(this.timeSchedule){
                        boolean successfulRemove = this.timeSchedule.removeEvent(taskID);
                        this.print("got decline taskID-->"+taskID+"\tremoved-->"+successfulRemove);
                    }
                }
                
                else if (m.type == "offer"){
                    this.offers.add(m);
                }

                else if (m.type.equals(new String("inform"))) {
                    // TA informs SA when its done with a task.
                    String informVal = this.parseMessage(m, "informVal")[0]; 

                    if (informVal.equals(new String("done"))) {
                        // current solution does not receive 'done'-msg
                        //TODO add code if implementing it that way
                        synchronized(this.timeSchedule){ this.timeSchedule.removeEvent(taskID); }
                        

                    }
                    else if (informVal.equals(new String("status"))) {
                        double newEndTime = Double.parseDouble(this.parseMessage(m, "", true)[2]);

                        synchronized(this.timeSchedule){
                            if ( this.timeSchedule.isNewEndTimePossible(taskID, newEndTime) ){
                                this.timeSchedule.setNewEndTime(taskID, newEndTime);
                            }
                            else{
                                this.sendMessage(new Message(this.robotID, m.sender, "inform", taskID+this.separator+"abort"));
                                this.timeSchedule.abortEvent(taskID);
                            }
                        }
                    }

                    else if (informVal.equals(new String("abort"))) {
                        synchronized(this.timeSchedule){ this.timeSchedule.abortEvent(taskID); }
                    } 
                }
                
            }
        
            // System.out.println(this.robotID + " -- " + this.robotsInNetwork);
            try { Thread.sleep(100); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
    }

    /**simlpe print func with color and robotID included
     * @param s string to be printed
     */
    protected void print(String s){
        System.out.println("\033[0;35m"+this.robotID+"\t" + s + "\033[0m");
    }

}