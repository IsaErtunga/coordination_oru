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


    public StorageAgent(int r_id, Router router, double capacity, Pose startPos, long startTime){} // deprecated
    public StorageAgent(int r_id, Router router, double capacity, Pose startPos, Pose startPoseRight, long startTime, ReedsSheppCarPlanner mp){} // deprecated

    /**
     * Constructor with MP and oreState
     * @param r_id
     * @param router
     * @param capacity
     * @param startPos
     */
    public StorageAgent(int r_id, Router router, double capacity, double startOre, Pose startPos,long startTime, 
                        ReedsSheppCarPlanner mp, OreState oreState, HashMap<String, PoseSteering[]> pathStorage){}

    public StorageAgent(int r_id, Router router, long startTime, NewMapData mapInfo, OreState oreState,
                        HashMap<String, PoseSteering[]> pathStorage, ReedsSheppCarPlanner mp){  

        this.robotID = r_id;
        this.COLOR = "\033[1;33m";
        this.capacity = mapInfo.getCapacity(r_id);
        this.amount = mapInfo.getStartOre(r_id);
        this.initialPose = mapInfo.getPose(r_id);
        //this.generateMotionPlanner(yamlFileString, mapInfo.getTurningRad(4), mapInfo.getAgentSize(4)); // 4 is for TTA
        this.mp = mp;
        this.agentVelocity = mapInfo.getVelocity(4);
        this.TTAcapacity = mapInfo.getCapacity(4);

        this.orderNumber = 1;

        this.timeSchedule = new TimeScheduleNew(oreState, this.initialPose, this.capacity, this.amount);
        this.clockStartTime = startTime;
        this.occupancyPadding = 5.0;

        // settings
        this.TIME_WAITING_FOR_OFFERS = 10.0;
        this.taskCap = 2;
        this.ORE_LEVEL_LOWER = 0.2 * this.capacity < 60.0 ? 60.0 : 0.1 * this.capacity; // TTA cap 40.0 * 1.5
        this.ORE_LEVEL_UPPER = 0.8 * this.capacity;

        this.pStorage = pathStorage;

        this.print("initiated");
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

        this.sleep(1000);

        this.status();

    }

    /**
     * HandleOffers is called from a SA, to either accept the offer of a TA, or deny it.
     * @return the message that is the best offer
     */
    public Message handleOffers(int taskID) {
        Message bestOffer = new Message();
        int bestOfferVal = 0;
        boolean debug = true;
        if(debug) this.print("-- in handleOffers");

        ArrayList<Message> offersCopy = new ArrayList<Message>(this.offers);
        
        for (Message m : offersCopy) {
            String[] mParts = this.parseMessage( m, "", true); 
            if ( Integer.parseInt(mParts[0]) != taskID ) continue; // sort out offer not part of current auction(taskID)

            int offerVal = Integer.parseInt(mParts[1]);
            if(debug) this.print("\tofferVal-->"+offerVal);

            // ========= EXPERIMENTAL =========
            double tStart = Double.parseDouble(mParts[4]);
            double tEnd = Double.parseDouble(mParts[5]);
            double[] occupiedTimes = this.translateTAtaskTimesToOccupyTimes(tStart, tEnd, this.occupancyPadding);
            double startTime = occupiedTimes[0];
            double endTime = occupiedTimes[1];
            // ================================
            if(debug) this.print("\tisTaskPossible("+taskID+", "+startTime+", "+endTime+")-->"+(this.timeSchedule.isTaskPossible(taskID, startTime, endTime)));
            if( this.timeSchedule.isTaskPossible(taskID, startTime, endTime) ) {
                if(debug) this.print("\t\t"+offerVal+">"+bestOfferVal+"-->"+(offerVal > bestOfferVal));
                if (offerVal > bestOfferVal){
                    if(debug) this.print("\t\t\tnew retOffer with-->"+m.sender);
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
        else availabeOre = availabeOre >= this.TTAcapacity ? this.TTAcapacity : availabeOre; 

        // Create Task
        Task TTATask = generateTaskFromAuction(m, this.initialPose, availabeOre);

        // ========= EXPERIMENTAL =========
        double[] timeUsingResource = this.translateTAtaskTimesToOccupyTimes(TTATask, this.occupancyPadding);
        if ( !this.timeSchedule.isTaskPossible(TTATask.taskID, timeUsingResource[0], timeUsingResource[1]) ) return;    // task doesnt fit in schedule
        
        // Calculate offer
        int offerVal = this.calculateOffer(TTATask, m);
        if ( offerVal <= 0 ) return;
        
        // Send Offer
        this.sendMessage(this.generateOfferMessage(TTATask, offerVal, availabeOre));
        TTATask.startTime = timeUsingResource[0];
        TTATask.endTime = timeUsingResource[1];
        this.timeSchedule.addEvent(TTATask);
        // ================================
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
        //PoseSteering[] path = this.calculatePath(this.mp, TTAPos, this.startPose);
        PoseSteering[] path = this.getPath(this.pStorage, this.mp, TTAPos, this.initialPose);
        double pathDist = this.calculatePathDist(path);
        double pathTime = this.calculateDistTime(pathDist, this.agentVelocity);

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
        synchronized(this.timeSchedule){
            this.amount = this.timeSchedule.markEventDone(taskID);
            this.timeSchedule.removeEvent(taskID);
        }
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

    @Override
    protected void handleInformStatus(Message m){
        String updateSep = "::";
        String pairSep = ":";

        this.print("in handleInformStatus");
        this.timeSchedule.printSchedule(this.COLOR);

        String informInfo = (this.parseMessage(m, "informInfo")[0]);
        String[] newTimes = informInfo.split(updateSep);
        for ( int i=0; i<newTimes.length; i++ ){
            String[] updatePair = newTimes[i].split(pairSep);
            int taskID = Integer.parseInt( updatePair[0] );
            double newEndTime = Double.parseDouble( updatePair[1] ) + this.occupancyPadding;
    
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

    public void status2 () {
        while(true) {
            this.sleep(500);

            double oreLevel = this.timeSchedule.getLastOreState();

            if ( this.amount < 0.5*this.capacity ){ // EXPERIMENTAL get ore now //TODO only works without TTA
                Message bestOffer = this.offerService(this.getTime());
                if (!bestOffer.isNull) {
                    Task task = this.generateTaskFromOffer(bestOffer);

                    double[] occupiedTimes = this.translateTAtaskTimesToOccupyTimes(task, this.occupancyPadding);
                    task.startTime = occupiedTimes[0];
                    task.endTime = occupiedTimes[1];
                    
                    this.timeSchedule.addEvent(task);
                    // this.print("in status, task added. ---Schedule---");
                    // this.timeSchedule.printSchedule(this.COLOR);
                }
            }

            else if (oreLevel < 0.9 * capacity) { // plan future tasks
                // this.print("in status");
                // this.timeSchedule.printSchedule(this.COLOR);
                // if ( this.timeSchedule.getSize() > this.taskCap ) continue;

                Message bestOffer = this.offerService(this.getNextTime());

                if (!bestOffer.isNull) {
                    Task task = this.generateTaskFromOffer(bestOffer);

                    // ========= EXPERIMENTAL =========
                    double[] occupiedTimes = this.translateTAtaskTimesToOccupyTimes(task, this.occupancyPadding);
                    task.startTime = occupiedTimes[0];
                    task.endTime = occupiedTimes[1];
                    
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

    public void status () {
        double ORE_NOW_PERCENT = 0.4;
        double ORE_STATE_PERCENT = 0.4;
        double ORE_FUTURE_PERCENT = 0.85;
        while (true){
            this.sleep(3000);

            //if ( this.occupancyPadding > 2.0 ) this.occupancyPadding += -0.02;

            double lastOreState;
            double[] timeSlot;
            double nextStartTime;
            synchronized(this.timeSchedule){
                if ( this.timeSchedule.getSize() > this.taskCap) continue;
                //timeSlot = this.timeSchedule.getSlotToFill(this.occupancyPadding*2 +TIMESLOT_ADD, ORE_STATE_PERCENT*this.capacity);
            }

            double auctionTime = -1.0;
            if ( this.amount < 0.4*this.capacity ) { // if we really need ore now
                synchronized(this.timeSchedule){
                    auctionTime = this.timeSchedule.getNextEarliestTime(this.getTime()+10.0, this.occupancyPadding*2.2);
                }
                this.print("get ore now. auction request time-->"+auctionTime);
            
            }
            /* else if ( timeSlot.length > 1 ){ // if we really need ore at some point
                auctionTime = (timeSlot[1] - timeSlot[0])/2 + timeSlot[0];
                this.print("fill timeslot");

            } else if ( lastOreState < ORE_FUTURE_PERCENT*this.capacity ){ // can we plan to get ore in future ?
                synchronized(this.timeSchedule){ auctionTime = this.timeSchedule.getNextStartTime() + this.occupancyPadding; }
                this.print("plan future");

            } else { // dont hold auction
                Task abortTask;
                synchronized(this.timeSchedule){
                    abortTask = this.timeSchedule.getFirstOreStateFail();
                    if ( abortTask != null ) this.timeSchedule.abortEvent(abortTask.taskID);
                }
                if ( abortTask != null ){
                    this.sendMessage(new Message(this.robotID, abortTask.partner, "inform", abortTask.taskID+this.separator+"abort") );
                }

                this.print("dont hold auction");
            }
            */

            if ( auctionTime != -1.0 ){
                Message bestOffer = this.offerService(auctionTime);
                if (bestOffer.isNull == true) continue;

                Task task = this.generateTaskFromOffer(bestOffer);
                double[] occupiedTimes = this.translateTAtaskTimesToOccupyTimes(task, this.occupancyPadding);
                task.startTime = occupiedTimes[0];
                task.endTime = occupiedTimes[1];
                synchronized(this.timeSchedule){
                    this.timeSchedule.addEvent(task);
                    this.print("task added:");
                    this.timeSchedule.printSchedule(this.COLOR);
                }
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
