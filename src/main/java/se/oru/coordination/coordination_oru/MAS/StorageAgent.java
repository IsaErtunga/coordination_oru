package se.oru.coordination.coordination_oru.MAS;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

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
    protected String COLOR = "\033[1;33m";
    protected double TIME_WAITING_FOR_OFFERS = 5.0;
    private static int taskCap = 5;

    protected HashMap<String, PoseSteering[]> pStorage;

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
    public StorageAgent(int r_id, Router router, double capacity, double startOre, Pose startPos,long startTime, 
                        ReedsSheppCarPlanner mp, OreState oreState, HashMap<String, PoseSteering[]> pathStorage){
        this.print("STORAGE AGENT INITIATED");
        this.robotID = r_id;
        this.capacity = capacity;
        this.amount = startOre;
        this.startPose = startPos;
        this.timeSchedule = new TimeScheduleNew(oreState, startPos, this.capacity, this.amount);
        this.startTime = startTime;
        this.mp = mp;
        this.orderNumber = 1;

        this.pStorage = pathStorage;

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
            this.sleep(2000);

            double oreLevel = this.timeSchedule.getLastOreState();
            if ( this.amount < oreLevel && false ){} //TODO request ore fast


            else if ( false ){} //TODO check if we need more ore at some point in future


            else if (oreLevel < 0.8 * capacity) { // plan future tasks
                // this.print("in status");
                // this.timeSchedule.printSchedule(this.COLOR);
                if ( this.timeSchedule.getSize() > this.taskCap ) continue;

                Message bestOffer = this.offerService(this.getNextTime());

                if (!bestOffer.isNull) {
                    Task task = this.createTaskFromMessage(bestOffer);
                    this.timeSchedule.addEvent(task);
                    // this.print("in status, task added. ---Schedule---");
                    // this.timeSchedule.printSchedule(this.COLOR);
                }
                
            }
            else {
                this.print("not doing anything atm...");
            }
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

        this.sleep(300);

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
                    this.print("received offer from-->"+m.sender+"\ttaskID-->"+taskID+"\tofferVal-->"+this.parseMessage(m, "offerVal")[0]);
                    this.offers.add(m);
                }

                else if (m.type == "accept") {
                    // this.print("");
                    // this.timeSchedule.printSchedule(this.COLOR);
                    boolean eventAdded;
                    synchronized(this.timeSchedule){ eventAdded = this.timeSchedule.setEventActive(taskID, true); }
                    this.print("accept-msg, taskID-->"+taskID+"\twith robot-->"+m.sender+"\ttask added-->"+eventAdded);
                    if ( eventAdded == false ){
                        this.print("accept received but not successfully added. sending abort msg");
                        //this.sendMessage(new Message(this.robotID, m.sender, "inform", taskID+this.separator+"abort"));
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

                else if (m.type == "inform") {
                    this.handleInformMessage(m);
                }

                
            }
            // Changed sleep from 1000
            this.sleep(100);
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

            // int docId = this.robotID % 1000;
            // try {
            //     this.fp.write(this.getTime(), this.timeSchedule.getOreStateAtTime(this.getTime()), docId);
            // } catch (IOException e) {
            //     e.printStackTrace();
            // }

            //this.print("ORESTATE: " + this.timeSchedule.getOreStateAtTime(this.getTime()) + " AT TIME: "+ this.getTime());
            synchronized(this.timeSchedule){ this.timeSchedule.removeEvent(taskID); }
            
            if (oreChange > 0) this.addOre(oreChange);
            else this.dumpOre(oreChange);
            //synchronized(this.timeSchedule){ this.timeSchedule.printSchedule(this.COLOR); }

        }
        else if (informVal.equals(new String("status"))) { 
            // this.print("in status: ---SCHEDULE---");
            // this.timeSchedule.printSchedule(this.COLOR);

            double newEndTime = Double.parseDouble(this.parseMessage(m, "", true)[2]);
            Task taskToAbort = this.timeSchedule.updateTaskEndTimeIfPossible(taskID, newEndTime); // this function aborts task from schedule

            if ( taskToAbort != null ){
                this.sendMessage(new Message(this.robotID, taskToAbort.partner, "inform", taskToAbort.taskID+this.separator+"abort"));
                this.print("sending ABORT msg. taskID-->"+taskToAbort.taskID+"\twith-->"+taskToAbort.partner );
            }
            else {this.print("updated without conflict-->"+taskID +"\twith-->"+ m.sender);}
        }                   

        else if (informVal.equals(new String("abort"))) { //TODO remove task from schedule 
            // this.print("---SCHEDULE---");
            // this.timeSchedule.printSchedule(this.COLOR);

            this.timeSchedule.abortEvent(taskID);
            this.print("got ABORT MSG! taskID-->"+taskID+"\twith-->"+m.sender );
            
        } 
    }

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportAgent} calling this
     */
    public Message offerService(double startTime){
        boolean debug = false;
        ArrayList<Integer> receivers = this.getReceivers(this.robotID, this.robotsInNetwork, "TRANSPORT");

        if ( receivers.size() <= 0 ) return new Message();
        this.offers.clear();
        int taskID = this.sendCNPmessage(startTime, this.stringifyPose(this.startPose), receivers);

        if(debug) this.print("starting to wait for offers at time-->"+ String.format("%.2f",this.getTime())+"\tnrReceivers-->"+receivers.size() );

        double time = this.waitForAllOffersToCome( receivers.size(), taskID );  //locking function. wait for receivers

        if(debug) this.print("done waiting at time-->"+ String.format("%.2f",this.getTime()) +"\tamountInOffers-->"+this.offers.size());

        Message bestOffer = this.handleOffers(taskID); //extract best offer
        if ( bestOffer.isNull == true ){
            this.print("no good offer received, ---schedule--- time-->"+this.getTime());
            this.timeSchedule.printSchedule(this.COLOR);
            if(debug) this.print("no offers received");

            Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(taskID));
            this.sendMessage(declineMessage);
            return bestOffer;
        }

        Message acceptMessage = new Message(robotID, bestOffer.sender, "accept", Integer.toString(taskID) );
        this.sendMessage(acceptMessage);

        receivers.removeIf(i -> i==bestOffer.sender);
        if (receivers.size() > 0){
            Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(taskID));
            this.sendMessage(declineMessage);
        }
        return bestOffer;
    }

    /** This function is locking. will wait for offers from all recipients to arrive OR return on the time delay.
     * 
     * @param nrOfReceivers an int with the number of recipients of the cnp message sent. 
     * @return a double with the time it took before exiting this function.
     */
    protected double waitForAllOffersToCome(int nrOfReceivers, int taskID){
        double before = this.getTime();
        ArrayList<Message> offerListCopy;

        while ( this.getTime() - before <= this.TIME_WAITING_FOR_OFFERS){
            int offersReceived = 0;
            synchronized(this.offers){ offerListCopy = new ArrayList<Message>(this.offers); }

            for ( Message offer : offerListCopy ){
                if ( Integer.parseInt(this.parseMessage(offer, "taskID")[0]) == taskID ) offersReceived++;
            }
            
            if ( offersReceived >= nrOfReceivers ) return (this.getTime() - before);

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
        boolean debug = false;
        Message bestOffer = new Message();
        int bestOfferVal = 0;
        double bestEndTime = 0;

        if(debug) this.print("in handleOffers ---schedule---");
        if(debug) this.timeSchedule.printSchedule(this.COLOR);

        ArrayList<Message> offersCopy = new ArrayList<Message>(this.offers);
        
        for (Message m : offersCopy) {

            String[] mParts = this.parseMessage( m, "", true); 

            if(debug) this.print("m.taskID == taskID-->"+(Integer.parseInt(mParts[0]) == taskID) );

            if ( Integer.parseInt(mParts[0]) != taskID ) continue; // sort out offer not part of current auction(taskID)

            int offerVal = Integer.parseInt(mParts[1]);
            double startTime = Double.parseDouble(mParts[4]);
            double endTime = Double.parseDouble(mParts[5]);

            if(debug) this.print("isTaskPossible("+taskID+", "+String.format("%.2f",startTime)+", "+String.format("%.2f",endTime)+"-->"+(this.timeSchedule.isTaskPossible(taskID, startTime, endTime)));

            if( this.timeSchedule.isTaskPossible(taskID, startTime, endTime) ) {

                if(debug) this.print("\tofferVal("+offerVal+" > "+bestOfferVal+"-->"+(offerVal > bestOfferVal));

                if (offerVal > bestOfferVal){
                    bestOfferVal = offerVal;
                    bestOffer = new Message(m);
                    bestEndTime = endTime;
                }
                else if ( offerVal == bestOfferVal && endTime < bestEndTime ){
                    bestOfferVal = offerVal;
                    bestOffer = new Message(m);
                    bestEndTime = endTime;

                    if(debug) this.print("changed offer because better startTime, new EndTime-->"+endTime);
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
        // Check how much ore we can deliver.
        double availabeOre = this.timeSchedule.getLastOreState();
        if (availabeOre <= 0.0) return false;   
        else availabeOre = availabeOre >= 40.0 ? 40.0 : availabeOre; 

        // Create Task
        Task TTATask = createTaskFromServiceOffer(m, availabeOre);
        if ( !this.timeSchedule.isTaskPossible(TTATask) ) return false;    // task doesnt fit in schedule
        
        // Calculate offer
        int offerVal = this.calculateOffer(TTATask);
        if ( offerVal <= 0 ) return false;
        if (! this.timeSchedule.addEvent(TTATask) ) return false;
        
        // Send Offer
        this.sendMessage(this.createOfferMsgFromTask(TTATask, offerVal, availabeOre));
                
        return true;
    }


    /** When receiving a cnp-message, this function will create a task from that message.
     * Only used in {@link handleService}.
     * @param m the message from the auctioneer
     * @param ore the amount of ore the task is about
     * @return a Task with attributes extracted from m
     */
    protected Task createTaskFromServiceOffer(Message m, double ore){
        String[] mParts = this.parseMessage(m, "", true);
        Pose TTAPos = this.posefyString(mParts[2]);
        // PoseSteering[] path = this.calculatePath(this.mp, TTAPos, this.startPose);
        PoseSteering[] path = this.getPath(this.pStorage, this.mp, TTAPos, this.startPose);
        double pathDist = this.calculatePathDist(path);
        double pathTime = this.calculateDistTime(pathDist);

        double taskStartTime = Double.parseDouble(mParts[3]);;
        double endTime = taskStartTime + pathTime;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -ore, taskStartTime, endTime, pathDist, TTAPos, this.startPose);
    }

   /**
     * Calculates and returns offer based on distance and ore-level
     * @param t
     * @return offer
     */
    protected int calculateOffer(Task t){
        int offer;
        if (t.pathDist > 0.5) {

            double oreLevel = this.timeSchedule.getOreStateAtTime(t.endTime);
            double oreLevelPercentage = oreLevel/this.capacity;

            if (oreLevelPercentage > 0.8) {
                double tooMuchOreBonus = 1000 * oreLevelPercentage;
                offer = (int)tooMuchOreBonus + this.calcCDF(t.pathDist, 500);
            }
            else if (oreLevelPercentage < 0.05) {
                offer = 0;
            }
            else {
                offer = (int) (oreLevelPercentage * this.calcCDF(t.pathDist, 500));
            }
        }
        else {
            offer = 0;
        }
        // this.timeSchedule.printSchedule(this.COLOR);
        return offer;
        // if (t.pathDist <= 2.0) return 0;

        // double dist = 100.0 * 1.0 / t.pathDist;
        // double evaluatedCapacity = 200.0 * this.amount / this.capacity; 

        // return (int)(dist + evaluatedCapacity);
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
        System.out.println(this.COLOR+this.robotID+" TIME["+String.format("%.2f",this.getTime()) + "]\t" + s + "\033[0m");
    }

}
