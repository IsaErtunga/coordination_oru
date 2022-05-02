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

    public TransportAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec, NewMapData mapInfo,
                            Router router, long startTime, String yamlFileString){
        
        this.robotID = r_id;
        this.COLOR = "\033[0;32m";
        this.tec = tec;
        this.initialPose = mapInfo.getPose(r_id);

        this.setRobotSpeedAndAcc(this.agentVelocity, 20.0);
        this.rShape = mapInfo.getAgentSize(r_id);
        this.generateMotionPlanner(yamlFileString, mapInfo.getTurningRad(2), this.rShape);
        this.agentVelocity = mapInfo.getVelocity(2);

        this.taskCap = 4;
        this.capacity = mapInfo.getCapacity(r_id);
        this.TIME_WAITING_FOR_OFFERS = 3.0;

        this.clockStartTime = startTime;
        this.timeSchedule = new TimeScheduleNew(this.initialPose, this.capacity, mapInfo.getStartOre(r_id));

        this.print("initiated");
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

        this.taskExecutionThread();
    

    }
    
    /**
     * Function enabling TA to execute the tasks in its schedule. 
     * TODO Add functionality for knowing if robot managed to complete a task or not. 
     */
    @Override
    protected void taskExecutionThread () {
        while (true) {
            this.sleep(200);

            Task task = null;
            Pose prevToPose = null;
            
            synchronized(this.timeSchedule){
                prevToPose = this.timeSchedule.getPoseAtTime(-1.0); // get endPose of current mission
                task = this.timeSchedule.getNextEvent();
            }
            if (task == null) continue;
            Mission taskMission = this.createMission(task, prevToPose);

            double now = this.getTime();            
            double timeBeforeMissionStarts = task.startTime - now;
            if ( timeBeforeMissionStarts > 0.5 ){
                this.print("starting sleep for-->"+timeBeforeMissionStarts);
                this.sleep( (int)((timeBeforeMissionStarts-0.5)*1000.0) ); 
                this.print("done sleeping");
            }

            // start mission
            synchronized(this.tec){ this.tec.addMissions(taskMission); }
            this.timeSchedule.printSchedule(this.COLOR);
            this.print("starting mission taskID-->"+task.taskID+" with -->" +task.partner + "\tat time-->"+this.getTime()+"\ttaskStartTime-->"+task.startTime);
            this.print("from pose-->"+prevToPose.toString() +"\tto pose-->"+ task.toPose.toString());

            // send inform update if needed
            if ( timeBeforeMissionStarts < 0.0 ) { // if we started mission late

                double nextStartTime = task.endTime - task.startTime + now;
                task.startTime = now;
                task.endTime = nextStartTime;
                ArrayList<Task> newEndTimes;
                synchronized(this.timeSchedule){
                    newEndTimes = this.timeSchedule.update(nextStartTime);
                    this.timeSchedule.changeOreStateEndTime(task.taskID, nextStartTime);
                }
                newEndTimes.add(0, task);

                // send inform msgs informing of new times.
                this.sendInformStatusMessages( newEndTimes, (now - task.startTime > 0.0) ); 
            }

            // wait for mission to be done.
            this.waitUntilCurrentTaskComplete(300); // locking
            
            this.print("mission DONE taskID-->"+task.taskID+" with -->" +task.partner + "\tat time-->"+this.getTime()+"\ttaskEndTime-->"+task.endTime);
            Message doneMessage = new Message(this.robotID, task.partner, "inform", task.taskID + this.separator + "done" + "," + task.ore);
            this.sendMessage(doneMessage);

        }
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
                    synchronized(this.timeSchedule){ this.timeSchedule.printSchedule(this.COLOR); }
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
        //double pathDist = ourPose.distanceTo(SApos);
        double pathDist = this.calculatePathDist(this.calculatePath(this.mp, ourPose, SApos) );
        double pathTime = this.calculateDistTime(pathDist, this.agentVelocity) + time_padding;
        double auctionTimeRequest = Double.parseDouble( mParts[3] );

        double ourNextTimeAvailable;
        synchronized(this.timeSchedule){ ourNextTimeAvailable = this.timeSchedule.getNextStartTime(); }
        double ourPossibleTimeAtTask = ourNextTimeAvailable + pathTime;

        double taskStartTime;
        if ( auctionTimeRequest > ourPossibleTimeAtTask) taskStartTime = auctionTimeRequest - pathTime;
        else taskStartTime = ourNextTimeAvailable;
        double endTime = taskStartTime + pathTime;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -ore, taskStartTime, endTime, pathDist, ourPose, SApos);
    }

    /** handleService is called from within a TA, when a TA did a {@link offerService}
     * @param m the message with the service
     * @param robotID the robotID of this object
     * @return true if we send offer = we expect resp.
     */
    @Override
    protected void handleCNPauction(Message m){
        this.print("in handleCNPauction");
        double availabeOre;
        Pose pos;
        synchronized(this.timeSchedule){
            availabeOre = this.timeSchedule.getLastOreState();
            pos = this.timeSchedule.getNextPose();
        }
        if (availabeOre <= 0.01) return;

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

    @Override
    protected void handleDecline(int taskID, Message m){
        this.print("in handleDecline");

        synchronized(this.timeSchedule){ this.timeSchedule.removeEvent(taskID); }
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
        int oreEval = Math.abs(t.ore) > this.capacity-0.1 ? 1000 : (int)this.linearDecreasingComparingFunc(Math.abs(t.ore), this.capacity, this.capacity, 500.0);
        
        // dist evaluation [1000, 0]
        int distEval = (int)this.concaveDecreasingFunc(t.fromPose.distanceTo(t.toPose), 1000.0, 120.0); // [1000, 0]

        // time bonus [500, 0]
        double cnpEndTime = Double.parseDouble(this.parseMessage(m, "startTime")[0]);
        double upperTimeDiff = cnpEndTime <= 0.0 ? 60.0 : 30.0;
        int timeEval = (int)this.linearDecreasingComparingFunc(t.endTime, cnpEndTime, upperTimeDiff, 500.0);  

        //TODO how long we have to sleep is part of equation
        
        //TODO add eval for using middle space of map. using middle space is BAD

        this.print("with robot-->"+m.sender +"\t dist-->"+ String.format("%.2f",t.pathDist) 
                        +"\tdistance eval-->"+distEval
                        +"\t cnp endTime-->"+String.format("%.2f",cnpEndTime) 
                        +"\t timeDiff-->"+String.format("%.2f",Math.abs(cnpEndTime - t.startTime )) 
                        +"\ttime eval-->"+timeEval);
        return oreEval + distEval + timeEval;
    }   
}