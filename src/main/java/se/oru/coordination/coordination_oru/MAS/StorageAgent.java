package se.oru.coordination.coordination_oru.MAS;
import java.io.IOException;
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
    //Control parameters
    protected double TIME_WAITING_FOR_OFFERS = 3.0;
    protected String COLOR = "\033[1;33m";

    protected Pose startPose;
    protected Pose startPoseRight;
    protected double capacity;  // capacity of storage = max ore it can store in TONS
    protected double amount;    // the current amount it has stored in TONS

    protected ReedsSheppCarPlanner mp;
    protected TimeScheduleNew timeSchedule;
    protected long startTime;
    protected double oreStateThreshold = 15.0;
    protected int orderNumber;

    public boolean beingUsed = false;

    // for testing
    public StorageAgent(int id){this.robotID = id;}     
    public StorageAgent(int r_id, Router router, double capacity, Pose startPos){}

    /**
     * Constructor for Storage Agent
     * @param r_id
     * @param router
     * @param capacity
     * @param startPos
     */
    public StorageAgent(int r_id, Router router, double capacity, Pose startPos, long startTime){
        this.print("STORAGE AGENT INITIATED");
        this.robotID = r_id;
        this.capacity = capacity;
        this.amount = capacity / 2.0;
        this.startPose = startPos;
        this.timeSchedule = new TimeScheduleNew(startPos, this.capacity, this.amount);
        this.startTime = startTime;

        router.enterNetwork(this);

        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);
    }

    /**
     * Constructor for Storage Agent with motion planner (for calculating path.)
     * @param r_id
     * @param router
     * @param capacity
     * @param startPos
     */
    public StorageAgent(int r_id, Router router, double capacity, Pose startPos, Pose startPoseRight, long startTime, ReedsSheppCarPlanner mp){
        this.print("STORAGE AGENT INITIATED");
        this.robotID = r_id;
        this.capacity = capacity;
        // this.amount = capacity / 2.0;
        this.amount = 0.2 * capacity;
        this.startPose = startPos;
        this.startPoseRight = startPoseRight;
        this.timeSchedule = new TimeScheduleNew(startPos, this.capacity, this.amount);
        this.startTime = startTime;
        this.mp = mp;
        this.orderNumber = 1;

        router.enterNetwork(this);

        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);
    }

    /**
     * Constructor with MP and oreState
     * @param r_id
     * @param router
     * @param capacity
     * @param startPos
     */
    public StorageAgent(int r_id, Router router, double capacity, double startOre, Pose startPos, long startTime, ReedsSheppCarPlanner mp, OreState oreState){
        this.print("STORAGE AGENT INITIATED");
        this.robotID = r_id;
        this.capacity = capacity;
        this.amount = startOre;
        this.startPose = startPos;
        this.timeSchedule = new TimeScheduleNew(oreState, startPos, this.capacity, this.amount);
        this.startTime = startTime;
        this.mp = mp;
        this.orderNumber = 1;

        router.enterNetwork(this);

        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);
    }


    protected double getTime(){
        long diff = System.currentTimeMillis() - this.startTime;
        return (double)(diff)/1000.0;
    }

    protected double getNextTime(){
        double STARTUP_ADD = 5.0;
        double nextTime = this.timeSchedule.getNextStartTime();
        return nextTime == -1.0 ? this.getTime()+STARTUP_ADD : nextTime;
    }

    public void status () {
        while(true) {
            double oreLevel = this.timeSchedule.getLastOreState();
            if ( this.amount < oreLevel && false ){} //TODO request ore fast


            else if ( false ){} //TODO check if we need more or at some point in future


            else if (oreLevel < 0.8 * capacity) { // plan future tasks

                Message bestOffer = this.offerService(this.getNextTime());

                if (!bestOffer.isNull) {
                    Task task = this.createTaskFromMessage(bestOffer, true);
                    this.timeSchedule.addEvent(task);
                    this.timeSchedule.printSchedule(this.COLOR);
                }
            }
            this.sleep(2000);
        }      
    }

    public void start(){
        StorageAgent This = this;
        Thread listener = new Thread() {
            public void run() {
                This.listener();
            }
        };
        listener.start();

        this.sleep(1000);

        this.status();

    }

    public void addOre(double ore){
        ore = ore > 0.0 ? ore : -ore; // give ore + sign
        if (this.amount + ore <= this.capacity && ore>0) this.amount = this.amount + ore;
    }

    public double dumpOre(double reqAmount){
        reqAmount = reqAmount > 0.0 ? reqAmount : -reqAmount; // give reqAmount + sign
        
        if (this.amount - reqAmount < 0){
            double ret = this.amount;
            this.amount = 0.0;
            return ret;
        }

        this.amount = this.amount - reqAmount;
        return reqAmount;
    }

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

                else if (m.type == "echo"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                }

                else if (m.type == "offer"){
                    this.offers.add(m);
                }

                else if (m.type == "accept"){
                    boolean eventAdded;
                    synchronized(this.timeSchedule){ eventAdded = this.timeSchedule.setEventActive(taskID); }
                    if (eventAdded) { 
                        this.print("task added to schedule, taskID-->"+taskID);
                    }
                    else{ //TODO task not added, need to send abort to taskProvider.
                        this.sendMessage(new Message(this.robotID, m.sender, "inform", taskID+this.separator+"abort"));
                    } 
                } //TODO does nothing in our Scenario atm

                else if (m.type == "decline"){
                    synchronized(this.timeSchedule){
                        boolean successfulRemove = this.timeSchedule.removeEvent(taskID);
                        this.print("got decline taskID-->"+taskID+"\tremoved-->"+successfulRemove);
                    }
                }

                else if (m.type == "cnp-service"){
                    this.print("CNP SERVICE______________----______________________--");
                    this.handleService(m);
                }

                else if (m.type == "inform") {
                    this.handleInformMessage(m);
                }

                
            }
            // Changed sleep from 1000
            this.sleep(500);
        }

    }

    /** this function holds the logic for handeling messages with type 'inform'. 
     * 
     * @param m the message with the 'inform'-type.
     */
    protected void handleInformMessage(Message m){
        int taskID = Integer.parseInt(this.parseMessage(m, "taskID")[0]);
        String informVal = this.parseMessage(m, "informVal")[0];
        
        if (informVal.equals(new String("done"))) {
            double oreChange = Double.parseDouble(this.parseMessage(m, "", true)[2]);
            
            try {
                this.fp.write(this.getTime(), this.timeSchedule.getOreStateAtTime(this.getTime()));
            } catch (IOException e) {
                e.printStackTrace();
            }

            this.print("ORESTATE: " + this.timeSchedule.getOreStateAtTime(this.getTime()) + " AT TIME: "+ this.getTime());
            synchronized(this.timeSchedule){ this.timeSchedule.removeEvent(taskID); }

            if (oreChange > 0) this.addOre(oreChange);
            else this.dumpOre(oreChange);

            this.print("RECEIVED ORE ___-_________________________");
            this.timeSchedule.printSchedule(this.COLOR);

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

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportAgent} calling this
     */
    public Message offerService(double startTime){
        ArrayList<Integer> receivers = this.getReceivers(this.robotID, this.robotsInNetwork, "TRANSPORT");

        if ( receivers.size() <= 0 ) return new Message();
        
        this.offers.clear();
        int taskID = this.sendCNPmessage(startTime, this.stringifyPose(this.startPose), receivers);

        double time = this.waitForAllOffersToCome(receivers.size());  //locking function. wait for receivers
        this.print("time waited for offers-->"+time);

        Message bestOffer = this.handleOffers(taskID); //extract best offer

        if (!bestOffer.isNull){        
            // Send response: Mission to best offer sender, and deny all the other ones.
            Message acceptMessage = new Message(robotID, bestOffer.sender, "accept", Integer.toString(taskID) );
            this.sendMessage(acceptMessage);

            receivers.removeIf(i -> i==bestOffer.sender);

            if (receivers.size() > 0){
                // Send decline message to all the others. 
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
    public int sendCNPmessage(double startTime, String startPos, ArrayList<Integer> receivers) {
        // taskID & agentID & pos & startTime 
        String startTimeStr = Double.toString(startTime);
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
            double startTime = Double.parseDouble(parseMessage(m, "startTime")[0]);
            double endTime = Double.parseDouble(parseMessage(m, "endTime")[0]);
  
            if( this.timeSchedule.isTaskPossible(taskID, startTime, endTime) ) {
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
        
        double availabeOre;
        synchronized(this.timeSchedule){
            availabeOre = this.timeSchedule.getLastOreState();
        }
        
        if (availabeOre <= 0.01) return false;   //if we dont have ore dont act 
        else availabeOre = availabeOre >= 15.0 ? 15.0 : availabeOre;

        Task TTATask = this.createTaskFromServiceOffer(m, availabeOre, this.startPose);

        boolean taskPossible;
        synchronized(this.timeSchedule){ taskPossible = this.timeSchedule.isTaskPossible(TTATask); }
        if ( !taskPossible ) return false;    // task doesnt fit in schedule
        
        int offerVal = this.calculateOffer(TTATask);
        
        if (offerVal <= 0.01) return false;

        synchronized(this.timeSchedule){ this.timeSchedule.addEvent(TTATask); }
   
        Message response = this.createOfferMsgFromTask(TTATask, offerVal, availabeOre);
    
        // räkna ut ett bud och skicka det.
        this.sendMessage(response);
                
        return true;
    }


    /** When receiving a cnp-message, this function will create a task from that message.
     * Only used in {@link handleService}.
     * @param m the message from the auctioneer
     * @param ore the amount of ore the task is about
     * @return a Task with attributes extracted from m
     */
    protected Task createTaskFromServiceOffer(Message m, double ore, Pose startPos){
        String[] mParts = this.parseMessage(m, "", true);
        Pose TTAPos = this.posefyString(mParts[2]);
        PoseSteering[] path = this.calculatePath(this.mp, TTAPos, startPos);
        double pathDist = this.calculatePathDist(path);
        double pathTime = this.calculateDistTime(pathDist);

        double taskStartTime;
        synchronized(this.timeSchedule){ taskStartTime = this.timeSchedule.getNextStartTime(); }
        double endTime = taskStartTime + pathTime;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -ore, taskStartTime, endTime, pathDist, TTAPos, startPos);
    }

   /**
     * Calculates and returns offer based on distance and ore-level
     * @param t
     * @return offer
     */
    protected int calculateOffer(Task t){
        int offer;
        if (t.pathDist > 0.5) {
            double oreLevel = this.timeSchedule.getLastOreState();
            double oreLevelPercentage = oreLevel/this.capacity;
            if (oreLevelPercentage > 0.8) {
                double tooMuchOreBonus = 1000 * oreLevelPercentage;
                offer = (int)tooMuchOreBonus + this.calcCDF(t.pathDist);
            }
            else if (oreLevelPercentage < 0.05) {
                offer = 0;
            }
            else {
                this.print("ELSE");
                offer = (int)(((oreLevel/this.capacity)*100) + this.calcCDF(t.pathDist));
            }
        }
        else {
            offer = 0;
        }
        this.timeSchedule.printSchedule(this.COLOR);
        return offer;
    }   

    /** Used to generate a response message from a task. Called from {@link handleService}
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

    /**simlpe print func with color and robotID included
     * @param s string to be printed
     */
    protected void print(String s){
        System.out.println(this.COLOR+this.robotID+"\t" + s + "\033[0m");
    }

}
