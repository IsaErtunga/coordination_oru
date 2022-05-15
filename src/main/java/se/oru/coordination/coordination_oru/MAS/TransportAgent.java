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

        this.taskCap = 4;
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
            if ( scSize >= this.taskCap || abortTasks.size() > 0) continue;

            boolean isDAtask = lastOreState <= oreLevelThreshold;
            Message bestOffer = this.holdAuction(this.getNextTime(), isDAtask);
            if (bestOffer.isNull) continue;
    
            Task task = isDAtask ? this.generateTaskFromOffer(bestOffer) : this.generateTaskFromOffer(bestOffer, -lastOreState);
            boolean taskAdded;
            synchronized(this.timeSchedule){ taskAdded = this.timeSchedule.addEvent(task); }

            if ( taskAdded == false ){ // if false then task no longer possible, send abort msg to task partner
                this.print("in initialState: task NOT added with-->"+task.partner+"\t taskID-->"+task.taskID);
                this.sendMessage(new Message(this.robotID, task.partner, "inform", task.taskID+this.separator+"abort"));
            } else {
                this.print("task added in initialState");
            }
        }
    }

    public Message holdAuction(double taskStartTime, boolean isOreRequest) {
        Pose nextPose;
        ArrayList<Integer> receivers = isOreRequest ? this.getReceivers("DRAW") : this.getReceivers("STORAGE");
        if ( receivers.size() < 1 ) return new Message();
        synchronized(this.timeSchedule){ nextPose = this.timeSchedule.getNextPose(); }

        this.offers.clear();
        int taskID = this.sendCNPmessage(taskStartTime, this.stringifyPose(nextPose), receivers);
        this.waitForAllOffersToCome(receivers.size(), taskID);

        Message bestOffer = isOreRequest ? this.handleOffersDA(taskID) : this.handleOffersSA(taskID, taskStartTime); //extract best offer
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

    public Message handleOffersDA(int taskID) {
        Message bestOffer = new Message();
        int offerVal = 0;

        ArrayList<Message> offerCopy = new ArrayList<Message>(this.offers);
        for (Message m : offerCopy) {
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

    public Message handleOffersSA(int taskID, double askingStartTime) {
        Message bestOffer = new Message();
        int offerVal = 0;

        ArrayList<Message> offerCopy = new ArrayList<Message>(this.offers);
        offerCopy.sort((o1, o2) -> Integer.parseInt(this.parseMessage(o2, "offerVal")[0]) - Integer.parseInt(this.parseMessage(o1, "offerVal")[0]) );
    
        for (Message m : offerCopy) {
            double taskStartTime = Double.parseDouble(parseMessage(m, "startTime")[0]);
            double endTime = Double.parseDouble(parseMessage(m, "endTime")[0]);
            //this.print("--handleOffersSA start->"+taskStartTime+" --- end->"+endTime);
            
            boolean taskPossible;
            synchronized(this.timeSchedule){ taskPossible = this.timeSchedule.isTaskPossible(taskID, taskStartTime, endTime); }
            //this.print("--handleOffersSA taskpossible->"+taskPossible);

            if (taskPossible) {
                String[] mParts = this.parseMessage( m, "", true); // sort out offer not part of current auction(taskID)
                //this.print("--handleOffersSA taskid correct->"+(Integer.parseInt(mParts[0]) == taskID));

                if ( Integer.parseInt(mParts[0]) == taskID ){
                    int val = Integer.parseInt(mParts[1]);

                     if (val > offerVal){
                        //this.print("--handleOffersSA secondIF");
                        offerVal = val;
                        bestOffer = new Message(m);
                    }
                }
            }
        }
        return bestOffer;
    }
    

    /**
   

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

/*
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
        this.TIME_WAITING_FOR_OFFERS = 4.0;

        this.clockStartTime = startTime;
        this.timeSchedule = new TimeScheduleNew(this.initialPose, this.capacity, mapInfo.getStartOre(r_id));
        this.LOAD_DUMP_TIME = 15.0 * 5.6 / this.agentVelocity;

        this.fp = fp;

        this.robotBreakdownTestProb = mapInfo.getRobotBreakdownTestProb();

        this.print("initiated");
        this.print("loadDump time-->"+this.LOAD_DUMP_TIME);
        // enter network and broadcast our id to others.
        router.enterNetwork(this);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);
                

    }

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
            if ( scSize >= this.taskCap || abortTasks.size() > 0) continue;

            boolean isDAtask = lastOreState <= oreLevelThreshold;
            Message bestOffer = this.holdAuction(this.getNextTime(), isDAtask);
            if (bestOffer.isNull) continue;
    
            Task task = isDAtask ? this.generateTaskFromOffer(bestOffer) : this.generateTaskFromOffer(bestOffer, -lastOreState);
            boolean taskAdded;
            synchronized(this.timeSchedule){ taskAdded = this.timeSchedule.addEvent(task); }

            if ( taskAdded == false ){ // if false then task no longer possible, send abort msg to task partner
                this.print("in initialState: task NOT added with-->"+task.partner+"\t taskID-->"+task.taskID);
                this.sendMessage(new Message(this.robotID, task.partner, "inform", task.taskID+this.separator+"abort"));
            } else {
                this.print("task added in initialState");
            }
        }
    }

    public Message holdAuction(double taskStartTime, boolean isOreRequest) {
        Pose nextPose;
        ArrayList<Integer> receivers = isOreRequest ? this.getReceivers("DRAW") : this.getReceivers("STORAGE");
        if ( receivers.size() < 1 ) return new Message();
        synchronized(this.timeSchedule){ nextPose = this.timeSchedule.getNextPose(); }

        this.offers.clear();
        int taskID = this.sendCNPmessage(taskStartTime, this.stringifyPose(nextPose), receivers);
        this.waitForAllOffersToCome(receivers.size(), taskID);

        Message bestOffer = isOreRequest ? this.handleOffersDA(taskID) : this.handleOffersSA(taskID, taskStartTime); //extract best offer
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

    public Message handleOffersDA(int taskID) {
        Message bestOffer = new Message();
        int offerVal = 0;

        ArrayList<Message> offerCopy = new ArrayList<Message>(this.offers);
        for (Message m : offerCopy) {
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

    public Message handleOffersSA(int taskID, double askingStartTime) {
        double BETTER_TIME_LOWER_OFFER_THRESHOLD = 1.2;
        Message bestOffer = new Message();
        int offerVal = 0;
        double nearAskingTime = 999.0;

        ArrayList<Message> offerCopy = new ArrayList<Message>(this.offers);
        offerCopy.sort((o1, o2) -> Integer.parseInt(this.parseMessage(o2, "offerVal")[0]) - Integer.parseInt(this.parseMessage(o1, "offerVal")[0]) );
    
        for (Message m : offerCopy) {
            double taskStartTime = Double.parseDouble(parseMessage(m, "startTime")[0]);
            double endTime = Double.parseDouble(parseMessage(m, "endTime")[0]);
            //this.print("--handleOffersSA start->"+taskStartTime+" --- end->"+endTime);

            if ( taskStartTime - askingStartTime < 0.0 ) continue;
            
            boolean taskPossible;
            synchronized(this.timeSchedule){ taskPossible = this.timeSchedule.isTaskPossible(taskID, taskStartTime, endTime); }
            //this.print("--handleOffersSA taskpossible->"+taskPossible);

            if (taskPossible) {
                String[] mParts = this.parseMessage( m, "", true); // sort out offer not part of current auction(taskID)
                //this.print("--handleOffersSA taskid correct->"+(Integer.parseInt(mParts[0]) == taskID));

                if ( Integer.parseInt(mParts[0]) == taskID ){
                    int val = Integer.parseInt(mParts[1]);

                    if ( (int)(BETTER_TIME_LOWER_OFFER_THRESHOLD * val) > offerVal && nearAskingTime > taskStartTime - askingStartTime ){
                        //this.print("--handleOffersSA firstIF");

                        nearAskingTime = taskStartTime - askingStartTime;
                        offerVal = val;
                        bestOffer = new Message(m);

                    } else if (val > offerVal){
                        //this.print("--handleOffersSA secondIF");
                        nearAskingTime = taskStartTime - askingStartTime;
                        offerVal = val;
                        bestOffer = new Message(m);
                    }
                }
            }
        }
        return bestOffer;
    }
}
*/