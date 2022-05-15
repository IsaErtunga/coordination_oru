package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Collections;
import java.lang.Math;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;


public class TransportAgent extends MobileAgent{

    protected double TIME_WAITING_ORESTATE_CHANGE = 4.0;

    public TransportAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec, NewMapData mapInfo,
                            Router router, long startTime, ReedsSheppCarPlanner mp, FilePrinter fp){
        
        this.robotID = r_id;
        this.COLOR = "\033[0;32m";
        this.tec = tec;
        this.initialPose = mapInfo.getPose(r_id);

        this.DIST_WEIGHT = mapInfo.getWeights(r_id)[0];
        this.ORE_WEIGHT = mapInfo.getWeights(r_id)[1];
        this.TIME_WEIGHT = mapInfo.getWeights(r_id)[2];

        this.agentVelocity = mapInfo.getVelocity(2);
        this.setRobotSpeedAndAcc(this.agentVelocity, 20.0);
        this.rShape = mapInfo.getAgentSize(r_id);
        this.mp = mp;

        this.taskCap = 3;
        this.capacity = mapInfo.getCapacity(r_id);
        this.TIME_WAITING_FOR_OFFERS = 3.0;

        this.clockStartTime = startTime;
        this.timeSchedule = new TimeScheduleNew(this.initialPose, this.capacity, mapInfo.getStartOre(r_id));
        this.LOAD_DUMP_TIME = 0.0;//15.0 * 5.6 / this.agentVelocity;

        this.fp = fp;

        this.robotBreakdownTestProb = mapInfo.getRobotBreakdownTestProb();

        this.print("initiated");
        this.print("loadDump time-->"+this.LOAD_DUMP_TIME);
        // enter network and broadcast our id to others.
        router.enterNetwork(this);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);
                

    }

    /**
     * 
     */
    public void start(){
        TransportAgent This = this;

        this.addRobotToSimulation();

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

        Thread robotBreakThread = new Thread() {
            public void run() {
                This.breakRobotTest();
            }
        };
        robotBreakThread.start();

        this.stateHandler();
    }

    @Override
    protected void handleDecline(int taskID, Message m){
        this.print("in handleDecline");

        synchronized(this.timeSchedule){ this.timeSchedule.removeEvent(taskID); }
    }

    protected void initialState() {
        double oreLevelThreshold = 1.0;
        while (true) {
            this.sleep(200);

            ArrayList<Task> abortTasks = new ArrayList<Task>();
            double lastOreState;
            int scSize;
            synchronized(this.timeSchedule){
                scSize = this.timeSchedule.getSize();
                abortTasks = this.timeSchedule.fixBrokenSchedule();
                lastOreState = this.timeSchedule.getLastOreState();
            }

            for ( Task t : abortTasks ){
                this.sendMessage(new Message(this.robotID, t.partner, "inform", t.taskID+this.separator+"abort"));
            }

            if ( lastOreState <= oreLevelThreshold && scSize < this.taskCap){ // book task to get ore
                Message bestOffer = this.offerService(this.getNextTime()); // hold auction with DA's

                if (bestOffer.isNull) continue;
                
                Task task = this.generateTaskFromOffer(bestOffer);

                boolean taskAdded;
                synchronized(this.timeSchedule){ taskAdded = this.timeSchedule.addEvent(task); }

                if ( taskAdded == false ){ // if false then task no longer possible, send abort msg to task partner
                    this.print("in initialState: task NOT added with-->"+task.partner+"\t taskID-->"+task.taskID);
                    this.sendMessage(new Message(this.robotID, task.partner, "inform", task.taskID+this.separator+"abort"));
                } else {
                    this.print("task added in initialState");
                    //synchronized(this.timeSchedule){ this.timeSchedule.printSchedule(this.COLOR); }
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
        ArrayList<Integer> receivers = this.getReceivers("DRAW");
        if (receivers.size() <= 0) return new Message();
    
        Pose nextPose;
        synchronized(this.timeSchedule){ nextPose = this.timeSchedule.getNextPose(); }
        
        this.offers.clear();
        int taskID = this.sendCNPmessage(taskStartTime, this.stringifyPose(nextPose), receivers);
        this.waitForAllOffersToCome(receivers.size(), taskID);

        Message bestOffer = this.handleOffers(taskID); //extract best offer
        if ( bestOffer.isNull == false ){
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

    @Override
    protected Task generateTaskFromAuction(Message m, Pose ourPose, double ore){
        String[] mParts = this.parseMessage(m, "", true);
        double time_padding = 2.0;
        Pose SApos = this.posefyString(mParts[2]);

        double pathDist = this.basicPathDistEstimate(ourPose, SApos);
        double pathTime = this.calculateDistTime(pathDist, this.agentVelocity) + this.LOAD_DUMP_TIME + time_padding;
        double auctionTimeRequest = Double.parseDouble( mParts[3] );

        double ourNextTimeAvailable = this.getNextTime();
        double ourPossibleTimeAtTask = ourNextTimeAvailable + pathTime;

        double tStart = auctionTimeRequest > ourPossibleTimeAtTask ? auctionTimeRequest - pathTime : ourNextTimeAvailable;
        double tEnd = tStart + pathTime;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -ore, tStart, tEnd, pathDist, ourPose, SApos);
    }

    @Override 
    protected void handleCNPauction(Message m){
        TransportAgent TA = this;
        Thread cnpThread = new Thread() {
            public void run() {
                TA.handleCNPauctionThread(m);
            }
        };
        cnpThread.start();
    }

    /** handleService is called from within a TA, when a TA did a {@link offerService}
     * @param m the message with the service
     * @param robotID the robotID of this object
     * @return true if we send offer = we expect resp.
     */
    protected void handleCNPauctionThread(Message m){
        double availabeOre = 0.0;
        double now = this.getTime();
        while ( availabeOre < 0.1 && this.getTime() - now < this.TIME_WAITING_ORESTATE_CHANGE ){
            this.sleep(100);
            synchronized(this.timeSchedule){ availabeOre = this.timeSchedule.getLastOreState(); }
        }
        if (availabeOre <= 0.01) return;

        Pose pos;
        synchronized(this.timeSchedule){ pos = this.timeSchedule.getNextPose(); }
        Task SATask = this.generateTaskFromAuction(m, pos, availabeOre);

        boolean taskPossible;
        synchronized(this.timeSchedule){ taskPossible = this.timeSchedule.isTaskPossible(SATask); }
        if ( taskPossible == false ) return;    // task doesnt fit in schedule

        int offerVal = this.calculateOffer(SATask, m);
        if ( offerVal <= 0 ) return;

        boolean taskAdded;
        synchronized(this.timeSchedule){ taskAdded = this.timeSchedule.addEvent(SATask); }
        if ( taskAdded == false ) return;

        this.sendMessage(this.generateOfferMessage(SATask, offerVal, availabeOre));
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
        int oreEval = (int) (Math.abs(t.ore) > this.capacity-0.1 ? 1000*this.ORE_WEIGHT : this.linearDecreasingComparingFunc(Math.abs(t.ore), this.capacity, this.capacity, 500.0)*this.ORE_WEIGHT);
        
        // dist evaluation [1500, 0]
        int distEval = (int) (this.concaveDecreasingFunc(t.fromPose.distanceTo(t.toPose), 1000.0, 80.0, 300.0)*this.DIST_WEIGHT); // [1500, 0]

        // time bonus [500, 0]
        double oreRequestTime = Double.parseDouble(this.parseMessage(m, "startTime")[0]); // startTime of cnp-msg is when auctioneer wants ore
        int timeEval = (int) (this.linearDecreasingComparingFunc(t.endTime, oreRequestTime, 45.0, 1000.0)*this.TIME_WEIGHT);  
        
        this.print("with robot-->"+m.sender +"\t dist-->"+ String.format("%.2f",t.pathDist) 
                        +"\tdistance eval-->"+distEval
                        +"\t cnp endTime-->"+String.format("%.2f",oreRequestTime) 
                        +"\t timeDiff-->"+String.format("%.2f",Math.abs(oreRequestTime - t.endTime )) 
                        +"\ttime eval-->"+timeEval);

        return oreEval + distEval + timeEval;
    }   

    /* good calc func, to avoid long sleeps, little collition/deadlocks
        int oreEval = Math.abs(t.ore) > this.capacity-0.1 ? 1000 : (int)this.linearDecreasingComparingFunc(Math.abs(t.ore), this.capacity, this.capacity, 500.0);
        
        // dist evaluation [1000, 0]
        int distEval = (int)this.concaveDecreasingFunc(t.fromPose.distanceTo(t.toPose), 1000.0, 120.0); // [1000, 0]

        // time bonus [500, 0]
        double cnpEndTime = Double.parseDouble(this.parseMessage(m, "startTime")[0]);
        double upperTimeDiff = cnpEndTime <= 0.0 ? 60.0 : 30.0;
        int timeEval = (int)this.linearDecreasingComparingFunc(t.endTime, cnpEndTime, upperTimeDiff, 500.0);  

    */
}