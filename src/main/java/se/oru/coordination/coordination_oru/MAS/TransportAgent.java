package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Arrays;
import java.util.Collections;
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
    protected final int taskCap = 6;

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
        double STARTUP_ADD = 8.0;
        synchronized(this.timeSchedule){
            double nextTime = this.timeSchedule.getNextStartTime();
            return (int)(nextTime) == -1 ? STARTUP_ADD : nextTime;
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
        synchronized(this.tec){
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
            Pose prevToPose = null;
            
            synchronized(this.timeSchedule){
                prevToPose = this.timeSchedule.getPoseAtTime(-1.0);
                task = this.timeSchedule.getNextEvent();
            }
            if (task == null) continue;

            // update schedule to actual start of mission and send inform msg to all tasks affected.
            double now = this.getTime();
            double nextStartTime = task.endTime - task.startTime + now;
            synchronized(this.timeSchedule){
                this.timeSchedule.changeOreStateEndTime(task.taskID, nextStartTime);
                newEndTimes = this.timeSchedule.compressSchedule(nextStartTime);
                this.timeSchedule.printSchedule(this.COLOR);
            }
            this.print("starting mission taskID-->"+task.taskID+" with -->" +task.partner + "\tat time-->"+this.getTime()+"\ttaskStartTime-->"+task.startTime);
            this.print("from pose-->"+prevToPose.toString() +"\tto pose-->"+ task.toPose.toString());
            task.startTime = now;
            task.endTime = nextStartTime;
            newEndTimes.add(0, task);

            // start next task
            synchronized(this.tec){ this.tec.addMissions(this.createMission(task, prevToPose)); }

            // send inform msgs informing of new times.
            this.sendInformStatusMessages( newEndTimes, (now - task.startTime > 0.0) ); 
            
            // wait for mission to be done.
            this.waitUntilCurrentTaskComplete(this.tec, this.robotID, 100); // locking
            
            this.print("mission DONE taskID-->"+task.taskID+" with -->" +task.partner + "\tat time-->"+this.getTime()+"\ttaskEndTime-->"+task.endTime);
            Message doneMessage = new Message(this.robotID, task.partner, "inform", task.taskID + this.separator + "done" + "," + task.ore);
            this.sendMessage(doneMessage, false);

        }
    }

    protected void sendInformStatusMessages(ArrayList<Task> tasksToUpdate, boolean isLater){
        if ( isLater ) Collections.reverse(tasksToUpdate);
        String updateSep = "::";
        String pairSep = ":";

        HashMap<Integer, ArrayList<Task>> taskMap = new HashMap<Integer, ArrayList<Task>>();
        
        for ( Task t : tasksToUpdate ){
            ArrayList<Task> agentTasks = taskMap.get(t.partner);
            if ( agentTasks == null ) agentTasks = new ArrayList<Task>();
                
            agentTasks.add(t);
            taskMap.put(t.partner, agentTasks);
        }

        for (int key : taskMap.keySet()) {
            ArrayList<Task> tasksNewTime = taskMap.get(key);
            Task firstElement = tasksNewTime.remove(0);

            String messageBody = "0" +this.separator+ "status" +this.separator+ firstElement.taskID + pairSep + firstElement.endTime;

            for ( Task t : tasksNewTime ){
                messageBody = messageBody +updateSep+ t.taskID +pairSep+ t.endTime;
            }
            this.sendMessage(new Message(this.robotID, key, "inform", messageBody));
        }
    }

    protected void initialState() {
        double oreLevelThreshold = 1.0;
        while (true) {
            this.sleep(200);

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

                if (bestOffer.isNull) continue;
                
    
                Task task = this.createTaskFromMessage(bestOffer);

                boolean taskAdded;
                synchronized(this.timeSchedule){ taskAdded = this.timeSchedule.addEvent(task); }

                if ( taskAdded == false ){ // if false then task no longer possible, send abort msg to task partner
                    this.print("in initialState: task NOT added with-->"+task.partner+"\t taskID-->"+task.taskID);
                    this.sendMessage(new Message(this.robotID, task.partner, "inform", Integer.toString(task.taskID)+this.separator+"abort"));
                }

                // this.print("in initialState: --- schedule ---");
                // synchronized(this.timeSchedule){ this.timeSchedule.printSchedule(this.COLOR); }
            }

            else {
                //this.print("WAITING FOR TASK BY STORAGE AGENT");
            }

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

        int offerVal = this.calculateOffer(SATask, m);
        
        if ( offerVal <= 0.01 ) return false;

        synchronized(this.timeSchedule){ this.timeSchedule.addEvent(SATask); }

        this.sendMessage(this.createOfferMsgFromTask(SATask, offerVal, availabeOre));

        this.print("in handleService");
        this.timeSchedule.printSchedule(this.COLOR);
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
        double pathDist = taskStartPos.distanceTo(SApos);
        double pathTime = this.calculateDistTime(pathDist) + 5.0;

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
    protected int calculateOffer(Task t, Message m){
        if ( t.pathDist < 2.0 ) return 0;

        // ore eval [1000, 0]
        int oreEval = Math.abs(t.ore) > 14.9 ? 1000 : (int)this.linearDecreasingComparingFunc(Math.abs(t.ore), 15.0, 15.0, 500.0);
        
        // dist evaluation [1000, 0]
        int distEval = (int)this.concaveDecreasingFunc(t.fromPose.distanceTo(t.toPose), 1000.0, 120.0); // [1000, 0]

        // time bonus [500, 0]
        double cnpStartTime = Double.parseDouble(this.parseMessage(m, "startTime")[0]);
        double upperTimeDiff = cnpStartTime <= 0.0 ? 60.0 : 30.0;
        int timeEval = (int)this.linearDecreasingComparingFunc(t.startTime, cnpStartTime, upperTimeDiff, 500.0);  
        
        //TODO add eval for using middle space of map. using middle space is BAD

        this.print("with robot-->"+m.sender +"\t dist-->"+ String.format("%.2f",t.pathDist) 
                        +"\tdistance eval-->"+distEval
                        +"\t cnp starttime-->"+String.format("%.2f",cnpStartTime) 
                        +"\t timeDiff-->"+String.format("%.2f",Math.abs(cnpStartTime - t.startTime )) 
                        +"\ttime eval-->"+timeEval);
        return oreEval + distEval + timeEval;
       
    }   

    /**
     * 
     * @param startPose
     * @param endPose
     * @return
     */
    public Mission createMission(Task task, Pose prevToPose) {
        return new Mission(this.robotID, this.calculatePath( this.mp, prevToPose, task.toPose ));
    }


    /** this function holds the logic for handeling messages with type 'inform'. 
     * 
     * @param m the message with the 'inform'-type.
     */
    protected void handleInformMessage(Message m){
        int taskID = Integer.parseInt(this.parseMessage(m, "taskID")[0]);
        String informVal = this.parseMessage(m, "informVal")[0];

        if (informVal.equals(new String("status"))) this.handleStatusMessage(m);

        else if (informVal.equals(new String("abort"))) {
            this.print("got ABORT MSG! taskID-->"+taskID+"\twith-->"+m.sender );
            synchronized(this.timeSchedule){ this.timeSchedule.abortEvent(taskID); }
        } 
    }

    protected void handleStatusMessage(Message m){
        String updateSep = "::";
        String pairSep = ":";

        String informInfo = (this.parseMessage(m, "informInfo")[0]);
        String[] newTimes = informInfo.split(updateSep);
        for ( int i=0; i<newTimes.length; i++ ){
            String[] updatePair = newTimes[i].split(pairSep);
            int taskID = Integer.parseInt( updatePair[0] );
            double newEndTime = Double.parseDouble( updatePair[1] );
    
            Task taskToAbort = null;
            synchronized(this.timeSchedule) { taskToAbort = this.timeSchedule.updateTaskEndTimeIfPossible(taskID, newEndTime); }
            if ( taskToAbort != null ){
                this.sendMessage(new Message(this.robotID, taskToAbort.partner, "inform", taskToAbort.taskID+this.separator+"abort"));
                this.print("CONFLICT! sending ABORT msg. taskID-->"+taskID+"\twith-->"+m.sender );
            } else {
                this.print("updated without conflict-->"+taskID +"\twith-->"+ m.sender);
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
                int taskID = Integer.parseInt(this.parseMessage(m, "taskID")[0]);
                
                if (m.type == "hello-world"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                    this.sendMessage( new Message( m.receiver.get(0), m.sender, "echo", Integer.toString(taskID)));
                }
                
                else if ( m.type.equals(new String("goodbye-world")) ){ 
                    this.robotsInNetwork.remove(m.sender);

                }

                else if (m.type == "echo"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                }

                else if (m.type == "offer"){
                    this.print("received offer from-->"+m.sender+"\ttaskID-->"+taskID+"\tofferVal-->"+this.parseMessage(m, "offerVal")[0]);
                    this.offers.add(m);
                }

                else if (m.type == "accept") {
                    boolean eventAdded;
                    synchronized(this.timeSchedule){ eventAdded = this.timeSchedule.setEventActive(taskID); }
                    this.print("accept-msg, taskID-->"+taskID+"\twith robot-->"+m.sender+"\ttask added-->"+eventAdded);
                    if ( eventAdded == false ){
                        this.print("accept received but not successfully added. sending abort msg");
                        this.sendMessage(new Message(this.robotID, m.sender, "inform", taskID+this.separator+"abort"));
                    }

                } 

                else if (m.type == "decline"){
                    synchronized(this.timeSchedule){
                        boolean successfulRemove = this.timeSchedule.removeEvent(taskID);
                        this.print("got decline from-->"+m.sender+"\ttaskID-->"+taskID+"\tremoved-->"+successfulRemove);
                    }
                }

                else if (m.type == "cnp-service"){
                    this.handleService(m);
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
        System.out.println(this.COLOR+this.robotID+" TIME["+String.format("%.2f",this.getTime()) + "]\t" + s + "\033[0m");
    }

}