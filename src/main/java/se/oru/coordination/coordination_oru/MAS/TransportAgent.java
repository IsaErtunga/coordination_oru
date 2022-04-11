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


public class TransportAgent extends CommunicationAid{
    //Control parameters
    protected double TIME_WAITING_FOR_OFFERS = 3.0;
    protected String COLOR = "\033[0;32m";


    protected TrajectoryEnvelopeCoordinatorSimulation tec;
    protected ReedsSheppCarPlanner mp;

    protected Coordinate[] rShape;
    protected Pose startPose;
    protected final Double oreCap = 15.0;
    protected Double holdingOre = 0.0;
    protected final int taskCap = 4;

    protected TimeSchedule timeSchedule;
    protected long startTime;

    public ArrayList<Message> missionList = new ArrayList<Message>();


    public TransportAgent(int id){this.robotID = id;}   // for testing

    public TransportAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
                        ReedsSheppCarPlanner mp, Pose startPos, Router router){}

    public TransportAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
                        ReedsSheppCarPlanner mp, Pose startPos, Router router, long startTime){
            
                            System.out.println("#######################");
                            System.out.println(r_id +" -- constructor");
        
        this.robotID = r_id;
        this.tec = tec;
        this.mp = mp;
        this.startPose = startPos;
        this.startTime = startTime;

        this.timeSchedule = new TimeSchedule(startPos, 0.0);

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

        this.sleep(2000);

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
     * 
     * @param task
     * @return
     */
    protected Boolean isMissionDone (Task task) {
        // Distance from robots actual position to task goal pose
        return this.tec.getRobotReport(this.robotID).getPose().distanceTo(task.toPose) < 0.5;
    }

    /**
     * Function enabling TA to execute the tasks in its schedule. 
     * TODO Add functionality for knowing if robot managed to complete a task or not. 
     */
    protected void executeTasks () {
        while (true) {
            // Execute task while its schedule is not empty
            if (this.timeSchedule.getSize() > 0) {
                /* SCHEDULE: Only execute task if: 
                    * It is time to perform task. (if not wait, TODO do not just wait or something..)
                    * The task is marked as active.

                    * IF we start performing task long after startTime:
                        - update schedule to the delay
                    *   - inform receiving-end of the delay
                    * TODO: Calculate if we go over endTime.
                */
                Task task = this.timeSchedule.pop();
                if (task == null) {
                    continue;
                }

                this.print("starting mission with -->" +task.partner);
                this.tec.addMissions(this.createMission(task));
                
                // if robot managed to complete task 
                //String oreChange = task.partner<10000 ? Integer.toString(this.oreCap) : Integer.toString(-this.oreCap);
                while (!isMissionDone(task)) {
                    this.sleep(500);
                }
                
                Message doneMessage = new Message(this.robotID, task.partner, "inform", task.taskID + this.separator + "done" + "," + task.ore);
                this.sendMessage(doneMessage, false);

                // if robot didn't manage to complete task
            }
            try { Thread.sleep(500); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
    }

    protected void initialState() {
        double oreLevelThreshold = 0.0;
        while (true) {

            if ( false ){} //TODO check if we have an inconsistent schedule ore-wise

            else if ( this.timeSchedule.checkEndStateOreLvl() <= oreLevelThreshold ){ // book task to get ore

                Message bestOffer = this.offerService(this.getNextTime()); // hold auction with DA's

                if (bestOffer.isNull){ 
                    this.sleep(200);
                    continue;
                }
    
                Task task = this.createTaskFromMessage(bestOffer, true);
    
                if ( this.timeSchedule.add(task) == false ){ // if false then task no longer possible, send abort msg to task partner
                    this.sendMessage(new Message(this.robotID, task.partner, "abort", Integer.toString(task.taskID)));
                }

                this.print("--- schedule ---");
                this.timeSchedule.printSchedule(this.COLOR);
            }

            else {
                this.print("WAITING FOR TASK BY STORAGE AGENT");
            }

            this.sleep(500);
        }

    }

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportAgent} calling this
     */
    public Message offerService(double taskStartTime) {

        // Get correct receivers
        ArrayList<Integer> receivers = this.getReceivers(this.robotsInNetwork, "DRAW");

        if (receivers.size() <= 0) return new Message();
        
        this.offers.clear();
        String startPos = this.stringifyPose(this.timeSchedule.getLastToPose());
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
    
        for (Message m : this.offers) {
            // SCHEDULE: Extract startTime & endTime and see if it fits into schedule
            double taskStartTime = Double.parseDouble(parseMessage(m, "startTime")[0]);
            double endTime = Double.parseDouble(parseMessage(m, "endTime")[0]);
  
            if( this.timeSchedule.taskPossible(taskStartTime, endTime) ) {
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
        this.print("handleService - start");
        
        double availabeOre = this.timeSchedule.checkEndStateOreLvl();
        if (availabeOre <= 0.01) return false;   //if we dont have ore dont act 

        Pose pos = this.timeSchedule.getLastToPose();

        Task SATask = this.createTaskFromServiceOffer(m, availabeOre, pos);

        if ( !this.timeSchedule.taskPossible(SATask) ) return false;    // task doesnt fit in schedule

        int offerVal = this.calculateOffer(SATask);
        
        if ( offerVal <= 0.01 ) return false;

        if (! this.timeSchedule.add(SATask) ){
            this.print("not added!");
            return false;
        }
        this.print("--- schedule ---");
        this.timeSchedule.printSchedule(this.COLOR);

        this.print("handleService-SENDING OFFER");

        this.sendMessage(this.createOfferMsgFromTask(SATask, offerVal, availabeOre));

        return true;
    }

    /** When receiving a cnp-message, this function will create a task from that message.
     * Only used in {@link handleService}.
     * @param m the message from the auctioneer
     * @param ore the amount of ore the task is about
     * @return a Task with attributes extracted from m
     */
    protected Task createTaskFromServiceOffer(Message m, double ore, Pose pos){
        String[] mParts = this.parseMessage(m, "", true);

        Pose SApos = this.posefyString(mParts[2]);
        // PoseSteering[] path = this.calculatePath(pos, TApos);
        // double pathDist = this.calculatePathDist(path);
        // double pathTime = this.calculateDistTime(pathDist);
        double pathDist = pos.distanceTo(SApos);
        double pathTime = this.calculateDistTime(pathDist) + 10.0;

        //double startTime = Double.parseDouble(mParts[3]);
        double taskStartTime = this.timeSchedule.getNextStartTime();
        
        double endTime = taskStartTime + pathTime;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -ore, taskStartTime, endTime, pathTime, pos, SApos);
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
    
    
    protected int calculateOffer(Task t){
        if (t.pathDist <= 2.0) return 0;

        double dist = 100.0 * 1.0 / t.pathDist;

        return (int)(dist);
        
    }   

    /**
     * 
     * @param startPose
     * @param endPose
     * @return
     */
    public Mission createMission(Task task) {
        return new Mission(this.robotID, this.calculatePath(this.mp, task.fromPose, task.toPose));
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
                    this.timeSchedule.setTaskActive(taskID);
                }

                else if (m.type == "decline"){
                    //remove task from activeTasks
                    //SCHEDULE: remove reserved task from schedule
                    this.timeSchedule.remove(taskID);
                }

                else if (m.type == "cnp-service"){
                    this.handleService(m);
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
                        this.timeSchedule.remove(taskID);

                    }
                    else if (informVal.equals(new String("status"))) {
                        /* SCHEDULE: 
                            * if TA notice it will not be done in time, we get inform->status msg
                            * update schedule with new time and check if problem
                            * if 2 mission have big overlap then send ABORT msg to later mission.
                            * else all is good.
                        */ 
                        double newEndTime = Double.parseDouble(this.parseMessage(m, "ore")[0]); //TODO ore is 3rd element NOT ore in this case
                        if (newEndTime > this.timeSchedule.get(taskID).endTime) {
                            this.timeSchedule.update(taskID, newEndTime);
                        }
                    }

                    else if (informVal.equals(new String("abort"))) {
                        /* SCHEDULE:
                            * remove task from schedule
                        */
                        this.timeSchedule.remove(taskID);
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
        System.out.println(this.COLOR+this.robotID+"\t" + s + "\033[0m");
    }

}
