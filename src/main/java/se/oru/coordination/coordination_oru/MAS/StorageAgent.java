package se.oru.coordination.coordination_oru.MAS;
import java.beans.EventHandler;
import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

public class StorageAgent extends AuctioneerBidderAgent{

    protected double TTAagentSpeed;

    protected FilePrinter fp;
    protected double lowCapacityVolume = 1.0;


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

    public StorageAgent(int r_id, Router router, long startTime, NewMapData mapInfo, OreState oreState, FilePrinter fp){  

        this.robotID = r_id;
        this.COLOR = "\033[1;33m";
        this.capacity = mapInfo.getCapacity(r_id);
        this.amount = mapInfo.getStartOre(r_id);
        this.initialPose = mapInfo.getPose(r_id);

        this.agentVelocity = mapInfo.getVelocity(2);
        this.TTAcapacity = mapInfo.getCapacity(4);
        this.TTAagentSpeed = mapInfo.getVelocity(4);

        this.timeSchedule = new TimeScheduleNew(oreState, this.initialPose, this.capacity, this.amount);
        this.clockStartTime = startTime;

        //this.occupancyPadding = 7.0;
        this.occupancyPadding = (25.0 / this.agentVelocity) * 1.7;

        // settings
        this.LOAD_DUMP_TIME = 0.0;//15.0 * 5.6 / this.agentVelocity;
        this.TIME_WAITING_FOR_OFFERS = 8.0;
        this.taskCap = 3;

        // Testing
        this.fp = fp;
        this.lowCapacityVolume = mapInfo.getLowCapacityTest();

        this.print("initiated");
        this.print("Capacity: " + this.capacity);
        this.print("loadDump time-->"+this.LOAD_DUMP_TIME);
        router.enterNetwork(this);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);
    }

    public void start(){
        StorageAgent This = this;
        Thread listenerThread = new Thread() {
            public void run() {
                This.listener();
            }
        };
        listenerThread.start();

        Thread amountThread = new Thread() {
            public void run() {
                This.updateAmountThread();
            }
        };
        amountThread.start();

        if (this.lowCapacityVolume < 1.0) {
            Thread lowerCapacityTest = new Thread() {
                public void run() {
                    This.changeCapacity();
                }
            };
            lowerCapacityTest.start();
        }
       

        this.sleep(1000);

        int block = this.robotID / 1000;
        if ( block != 9  ) this.status();
    }

    /**
     * Stochastically update capacity
     */
    protected void changeCapacity() {
        // Sleep for 2 minutes
        int uniqueID = (this.robotID % 1000) % 100;
        if ( uniqueID != 1 ) return;
        this.sleep(1000 * 180);

        double newCapacity = this.capacity * this.lowCapacityVolume;

        this.capacity = newCapacity;
        this.timeSchedule.setCapacity(newCapacity);
    }

    protected void updateAmountThread(){
        while (true){
            this.sleep(500);
            synchronized(this.timeSchedule){ this.amount = this.timeSchedule.getAmount(); }
        }
    }

    /**
     * HandleOffers is called from a SA, to either accept the offer of a TA, or deny it.
     * @return the message that is the best offer
     */
    public Message handleOffers(int taskID) {
        Message bestOffer = new Message();
        int bestOfferVal = 0;
        boolean debug = false;

        ArrayList<Message> offersCopy = new ArrayList<Message>(this.offers);
        if(debug) this.print("-- in handleOffers");

        for (Message m : offersCopy) {
            String[] mParts = this.parseMessage( m, "", true); 
            if ( Integer.parseInt(mParts[0]) != taskID ) continue; // sort out offer not part of current auction(taskID)

            int offerVal = Integer.parseInt(mParts[1]);
            if(debug) this.print("\tofferVal-->"+offerVal+", with-->"+m.sender);

            // ========= EXPERIMENTAL =========
            double tStart = Double.parseDouble(mParts[4]);
            double tEnd = Double.parseDouble(mParts[5]);
            double[] occupiedTimes = this.translateTAtaskTimesToOccupyTimes(tStart, tEnd, this.occupancyPadding);
            double startTime = occupiedTimes[0];
            double endTime = occupiedTimes[1];

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
        // Create Task
        Task TTATask = generateTaskFromAuction(m, this.initialPose);
        if ( Math.abs(TTATask.ore) < this.TTAcapacity ) return;

        // ========= EXPERIMENTAL =========
        double padding = 6.0;//25.0 / this.TTAagentSpeed;
        double[] timeUsingResource = this.translateTAtaskTimesToOccupyTimes(TTATask, padding); // TApadding *2 = TTA padding, but /2 because holding resource shorter
        if ( !this.timeSchedule.isTaskPossible(TTATask.taskID, timeUsingResource[0], timeUsingResource[1]) ) return;    // task doesnt fit in schedule
        
        // Calculate offer
        int offerVal = this.calculateOffer(TTATask, m);
        if ( offerVal <= 0 ) return;
        
        // Send Offer
        this.sendMessage(this.generateOfferMessage(TTATask, offerVal, Math.abs(TTATask.ore)));
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
    protected Task generateTaskFromAuction(Message m, Pose ourPose){
        String[] mParts = this.parseMessage(m, "", true);
        Pose TTAPos = this.posefyString(mParts[2]);
        //PoseSteering[] path = this.calculatePath(this.mp, TTAPos, this.startPose);
        //PoseSteering[] path = this.getPath(this.pStorage, this.mp, TTAPos, this.initialPose);
        //double pathDist = this.calculatePathDist(path);

        Pose midWayPose = new Pose(TTAPos.getX(), 27.5, 0.0);
        double pathDist = this.basicPathDistEstimate(TTAPos, midWayPose) + this.basicPathDistEstimate(midWayPose, this.initialPose);
        double pathTime = this.calculateDistTime(pathDist, this.TTAagentSpeed);

        double taskStartTime = Double.parseDouble(mParts[3]);;
        double endTime = taskStartTime + pathTime;
        double availableOre;
        synchronized(this.timeSchedule){ availableOre = this.timeSchedule.getOreStateAtTime(endTime); }
        availableOre = availableOre > this.TTAcapacity ? this.TTAcapacity : availableOre;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -availableOre, taskStartTime, endTime, pathDist, TTAPos, this.initialPose);
    }

    @Override
    protected int calculateOffer(Task agentTask, Message autionMessage){
        if (agentTask.pathDist < 0.5) return 0;

        double oreLevel = this.timeSchedule.getOreStateAtTime(agentTask.endTime) - this.TTAcapacity;
        if ( oreLevel < 0.1*this.capacity ) return 0;

        int oreEval = (int)this.linearDecreasingComparingFunc(oreLevel, this.capacity, this.capacity, 1000);
       
        this.print("-- calculateOffer: offer->"+oreEval+" with agent->"+agentTask.partner);
        return oreEval;
    }   

    @Override
    protected void handleInformDone(int taskID, Message m){
        synchronized(this.timeSchedule){
            this.amount = this.timeSchedule.markEventDone(taskID);
            this.timeSchedule.removeEvent(taskID);
        }
        int docId = this.robotID % 1000;
        this.fp.logOreState(this.getTime(), this.amount, docId);
        this.print("currentOre -->"+this.amount);
    }

    @Override
    protected void handleAccept(int taskID, Message m){
        boolean eventAdded;
        synchronized(this.timeSchedule){
            this.print("adding task");
            //this.timeSchedule.printSchedule(this.COLOR);
            eventAdded = this.timeSchedule.setEventActive(taskID);
        }

        if ( eventAdded ){
            this.print("--handleAccept: event added with ->"+m.sender);
        } else {
            this.print("--handleAccept: event not succesfullt added! Sending abort msg to-->"+m.sender);
            this.sendMessage(new Message(this.robotID, m.sender, "inform", taskID+this.separator+"abort"));
        }
    }

    @Override
    protected void handleInformStatus(Message m){
        String updateSep = "::";
        String pairSep = ":";

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

    public void status () {
        while ( true ){
            this.sleep(1000);

            double lookOre;
            synchronized(this.timeSchedule){
                if ( this.timeSchedule.getSize() > this.taskCap) continue;
                // this.print("-- in status. about to get auction time");
                // this.timeSchedule.printSchedule(this.COLOR);
                lookOre = this.timeSchedule.getLowestOreAfterTime(this.getTime()+10.0+this.LOAD_DUMP_TIME);
            }

            double slotSize = this.occupancyPadding*3+this.LOAD_DUMP_TIME;
            double auctionTime = -1.0;
            while ( lookOre < 0.9*this.capacity && auctionTime == -1.0 ){
                if ( lookOre > 0.6 * this.capacity ) this.sleep(10000);
                double timeLookAfter = this.getTime()+10.0+this.LOAD_DUMP_TIME;
                double timeLookBefore = timeLookAfter + 90.0; // two minutes plan ahead
                synchronized(this.timeSchedule){
                    auctionTime = this.timeSchedule.getNextEarliestTime(slotSize, lookOre, timeLookAfter, timeLookBefore, this.TAcapacity );
                }

                lookOre += 0.1 * this.capacity;
            }

            if ( auctionTime != -1.0 ){
                //this.print("requesting ore with auction time-->"+auctionTime);
                // ========= time auction creation
                double startTime = this.getTime();
                Message bestOffer = this.offerService(auctionTime);
                if (bestOffer.isNull == true) continue;
                // ==============================

                Task task = this.generateTaskFromOffer(bestOffer);
                
                double timeElapsed = this.getTime() - startTime;
                this.fp.addWaitingTimeMeasurment("auctionToTask", timeElapsed, this.robotID);
                
                double[] occupiedTimes = this.translateTAtaskTimesToOccupyTimes(task, this.occupancyPadding);
                task.startTime = occupiedTimes[0];
                task.endTime = occupiedTimes[1];
                synchronized(this.timeSchedule){
                    this.timeSchedule.addEvent(task);
                    // this.print("task added:");
                    // this.timeSchedule.printSchedule(this.COLOR);
                }
            } else {
                this.print("-- in status : all seems good...");
                this.sleep(3000);
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
        if (!bestOffer.isNull){        
            Message acceptMessage = new Message(robotID, bestOffer.sender, "accept", Integer.toString(taskID) );
            this.sendMessage(acceptMessage);
            receivers.removeIf(i -> i==bestOffer.sender);
        }
        if (receivers.size() > 0){
            Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(taskID));
            this.sendMessage(declineMessage);
        }
        return bestOffer;
    }
}
