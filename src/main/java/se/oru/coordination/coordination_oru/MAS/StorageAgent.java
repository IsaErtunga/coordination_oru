package se.oru.coordination.coordination_oru.MAS;
import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;



public class StorageAgent extends AuctioneerBidderAgent{
    //Control parameters
    protected HashMap<String, PoseSteering[]> pStorage;

    protected Pose startPoseRight;
    protected double ORE_LEVEL_LOWER;
    protected double ORE_LEVEL_UPPER;
    protected int orderNumber;

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
        this.initialPose = startPos;
        this.timeSchedule = new TimeScheduleNew(startPos, this.capacity, this.amount);
        this.clockStartTime = startTime;

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
        this.initialPose = startPos;
        this.startPoseRight = startPoseRight;
        this.timeSchedule = new TimeScheduleNew(startPos, this.capacity, this.amount);
        this.clockStartTime = startTime;
        this.mp = mp;
        this.orderNumber = 1;
        this.COLOR = "\033[1;33m";

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
        this.initialPose = startPos;
        this.timeSchedule = new TimeScheduleNew(oreState, startPos, this.capacity, this.amount);
        this.clockStartTime = startTime;
        this.mp = mp;
        this.orderNumber = 1;
        this.COLOR = "\033[1;33m";

        // settings
        this.TIME_WAITING_FOR_OFFERS = 5.0;
        this.taskCap = 10;
        this.ORE_LEVEL_LOWER = 0.1 * this.capacity < 60.0 ? 60.0 : 0.1 * this.capacity; // TTA cap 40.0 * 1.5
        this.ORE_LEVEL_UPPER = 0.8 * this.capacity;

        this.pStorage = pathStorage;

        router.enterNetwork(this);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);
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
        if (this.amount + ore <= this.capacity+0.1 && ore>0) this.amount = this.amount + ore;
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

    /**
     * HandleOffers is called from a SA, to either accept the offer of a TA, or deny it.
     * @return the message that is the best offer
     */
    public Message handleOffers(int taskID) {
        Message bestOffer = new Message();
        int bestOfferVal = 0;

        ArrayList<Message> offersCopy = new ArrayList<Message>(this.offers);
        
        for (Message m : offersCopy) {
            String[] mParts = this.parseMessage( m, "", true); 
            if ( Integer.parseInt(mParts[0]) != taskID ) continue; // sort out offer not part of current auction(taskID)

            int offerVal = Integer.parseInt(mParts[1]);
            double startTime = Double.parseDouble(mParts[4]);
            double endTime = Double.parseDouble(mParts[5]);

            if( this.timeSchedule.isTaskPossible(taskID, startTime, endTime) ) {
                if (offerVal > bestOfferVal){
                    bestOfferVal = offerVal;
                    bestOffer = new Message(m);
                }
            }
        }
        return bestOffer;
    }

    @Override
    protected void handleCNPauction(Message m){
        double availabeOre = this.timeSchedule.getLastOreState();
        if (availabeOre <= 0.0) return;   
        else availabeOre = availabeOre >= 40.0 ? 40.0 : availabeOre; 

        // Create Task
        Task TTATask = generateTaskFromAuction(m, this.initialPose, availabeOre);
        if ( !this.timeSchedule.isTaskPossible(TTATask) ) return;    // task doesnt fit in schedule
        
        // Calculate offer
        int offerVal = this.calculateOffer(TTATask, m);
        if ( offerVal <= 0 ) return;
        if (! this.timeSchedule.addEvent(TTATask) ) return;
        
        // Send Offer
        this.sendMessage(this.generateOfferMessage(TTATask, offerVal, availabeOre));
    }

    /**
     * When receiving a cnp-message, this function will create a task from that message.
     * Only used in {@link handleService}.
     * @param m the message from the auctioneer
     * @param ore the amount of ore the task is about
     * @return a Task with attributes extracted from m
     */
    @Override
    protected Task generateTaskFromAuction(Message m, Pose ourPose, double ore){
        String[] mParts = this.parseMessage(m, "", true);
        Pose TTAPos = this.posefyString(mParts[2]);
        // PoseSteering[] path = this.calculatePath(this.mp, TTAPos, this.startPose);
        PoseSteering[] path = this.getPath(this.pStorage, this.mp, TTAPos, this.initialPose);
        double pathDist = this.calculatePathDist(path);
        double pathTime = this.calculateDistTime(pathDist);

        double taskStartTime = Double.parseDouble(mParts[3]);;
        double endTime = taskStartTime + pathTime;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -ore, taskStartTime, endTime, pathDist, TTAPos, this.initialPose);
    }

    @Override
    protected int calculateOffer(Task agentTask, Message autionMessage){
        int offer;
        if (agentTask.pathDist > 0.5) {

            double oreLevel = this.timeSchedule.getOreStateAtTime(agentTask.endTime);
            double oreLevelPercentage = oreLevel/this.capacity;

            if (oreLevelPercentage > 0.8) {
                double tooMuchOreBonus = 1000 * oreLevelPercentage;
                offer = (int)tooMuchOreBonus + this.calcCDF(agentTask.pathDist, 500);
            }
            else if (oreLevelPercentage < 0.05) {
                offer = 0;
            }
            else {
                offer = (int) (oreLevelPercentage * this.calcCDF(agentTask.pathDist, 500));
            }
        }
        else {
            offer = 0;
        }
        return offer;
    }   

    @Override
    protected void handleInformDone(int taskID, Message m){
        double oreChange = this.timeSchedule.getEvent(taskID).ore;
        double currentOre = this.timeSchedule.getOreStateAtTime(this.getTime());
        this.timeSchedule.removeEvent(taskID);
        this.amount += oreChange;
        this.print("currentOre -->"+this.amount);
        /*
        double oreChange = Double.parseDouble(this.parseMessage(m, "informInfo")[0]);
        oreChange = oreChange * -1;

        synchronized(this.timeSchedule){ this.timeSchedule.removeEvent(taskID); }
        this.print("---SCHEDULE---");
        this.timeSchedule.printSchedule(this.COLOR);
        
        if (oreChange > 0) this.addOre(oreChange);
        else this.dumpOre(oreChange);
        */
    }

    @Override
    protected void handleAccept(int taskID, Message m){
        boolean eventAdded;
        synchronized(this.timeSchedule){ eventAdded = this.timeSchedule.setEventActive(taskID, true); }
        this.print("accept-msg, taskID-->"+taskID+"\twith robot-->"+m.sender+"\ttask added-->"+eventAdded);
        if ( eventAdded == false ){
            this.print("accept received but not successfully added. sending abort msg");
            this.sendMessage(new Message(this.robotID, m.sender, "inform", taskID+this.separator+"abort"));
        }
    }

    public void status () {
        while(true) {
            this.sleep(2000);

            double oreLevel = this.timeSchedule.getLastOreState();

            if (oreLevel < 0.9 * capacity) { // plan future tasks
                // this.print("in status");
                // this.timeSchedule.printSchedule(this.COLOR);
                if ( this.timeSchedule.getSize() > this.taskCap ) continue;

                Message bestOffer = this.offerService(this.getNextTime());

                if (!bestOffer.isNull) {
                    Task task = this.generateTaskFromOffer(bestOffer);
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

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportAgent} calling this
     */
    public Message offerService(double startTime){
        ArrayList<Integer> receivers = this.getReceivers("TRANSPORT");

        if ( receivers.size() <= 0 ) return new Message();
        this.offers.clear();
        int taskID = this.sendCNPmessage(startTime, this.stringifyPose(this.initialPose), receivers);

        this.waitForAllOffersToCome( receivers.size(), taskID );  //locking function. wait for receivers

        Message bestOffer = this.handleOffers(taskID); //extract best offer
        if ( bestOffer.isNull == true ){
            this.print("no good offer received");

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
}
