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

    protected HashMap<Integer, Integer> cascadeTaskPair = new HashMap<Integer, Integer>();

    public TransportAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec, NewMapData mapInfo,
                            Router router, long startTime, ReedsSheppCarPlanner mp){
        
        this.robotID = r_id;
        this.COLOR = "\033[0;32m";
        this.tec = tec;
        this.initialPose = mapInfo.getPose(r_id);
        
        this.agentVelocity = mapInfo.getVelocity(2);
        this.setRobotSpeedAndAcc(this.agentVelocity, 20.0);
        this.rShape = mapInfo.getAgentSize(r_id);
        this.mp = mp;

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

        // Thread stateThread = new Thread() {
        //     public void run() {
        //         This.initialState();
        //     }
        // };
        // stateThread.start();

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
            double beforeMissionPlan = this.getTime();
            Mission taskMission = this.createMission(task, prevToPose);
            this.print("time taken to plan-->"+(this.getTime()-beforeMissionPlan));

            double now = this.getTime();            
            double timeBeforeMissionStarts = task.startTime - now;
            if ( timeBeforeMissionStarts > 0.5 ){
                this.print("starting sleep for-->"+timeBeforeMissionStarts);
                this.sleep( (int)((timeBeforeMissionStarts-0.5)*1000.0) ); 
                this.print("done sleeping for-->"+timeBeforeMissionStarts);
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

            // if we finnish a SA mission, remove the cascade pair from cascadeTaskPair
            if ( (task.partner % 1000)/100 == 3 ) this.cascadeTaskPair.remove(task.taskID);
            
        }
    }

    /** //TODO looks good for now. will use timeSchedule.evaluateTimeSlot() in future
     * 
     * HandleOffers is called from a SA, to either accept the offer of a TA, or deny it.
     * @return the message that is the best offer
     */
    public Message handleOffers(int taskID) {
        Message bestOffer = new Message();
        int offerVal = 0;
        ArrayList<Message> offersCopy = new ArrayList<Message>(this.offers);
        
        for (Message m : offersCopy) {
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

    
    protected Task generateTaskFromAuction(Message m, Pose ourPose, double ore, double ourNextTimeAvailable){
        String[] mParts = this.parseMessage(m, "", true);
        double time_padding = 2.0;
        Pose SApos = this.posefyString(mParts[2]);
        //double pathDist = ourPose.distanceTo(SApos);
        double pathDist = this.calculatePathDist(this.calculatePath(this.mp, ourPose, SApos) );
        double pathTime = this.calculateDistTime(pathDist, this.agentVelocity) + time_padding;
        double auctionTimeRequest = Double.parseDouble( mParts[3] );

        double ourPossibleTimeAtTask = ourNextTimeAvailable + pathTime;

        double taskStartTime = ourNextTimeAvailable;
        //if ( auctionTimeRequest > ourPossibleTimeAtTask) taskStartTime = auctionTimeRequest - pathTime;
        //else taskStartTime = ourNextTimeAvailable;
        double endTime = taskStartTime + pathTime;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -ore, taskStartTime, endTime, pathDist, ourPose, SApos);
    }    

    @Override 
    protected void handleCNPauction(Message m){
        TransportAgent TA = this;
        Thread cnpThread = new Thread() {
            public void run() {
                TA.handleAuctionCascade(m);
            }
        };
        cnpThread.start();
    }

    protected void handleAuctionCascade(Message m){
        // får aution från storage // taskID 10
        Pose pos;
        double ourNextTimeAvailable;
        double nextStartTime;
        synchronized(this.timeSchedule){
            nextStartTime = this.timeSchedule.getNextStartTime();
            pos = this.timeSchedule.getNextPose();
        }
        nextStartTime = nextStartTime < this.getTime() ? this.getTime()+5.0 : nextStartTime;

        // håll i auction me DA (lägg in i reserved)
        Message bestOfferDA = this.offerService(pos, nextStartTime);
        if ( bestOfferDA.isNull == true ) return;
        Task taskDA = this.generateTaskFromOffer(bestOfferDA, false);

        Task taskSA = this.generateTaskFromAuction(m, taskDA.toPose, taskDA.ore, taskDA.endTime);
        boolean taskPossible;
        synchronized(this.timeSchedule){ taskPossible = this.timeSchedule.isTaskPossible(taskSA); }
        if ( taskPossible == false ) return;    // task doesnt fit in schedule

        int offerVal = this.calculateOffer(taskSA, taskDA, m);
        if ( offerVal <= 0 ) return;

        boolean SAtaskAdded;
        boolean DAtaskAdded;
        synchronized(this.timeSchedule){
            SAtaskAdded = this.timeSchedule.addEvent(taskSA); 
            DAtaskAdded = this.timeSchedule.addEvent(taskDA); 
        }
        if ( SAtaskAdded == false || DAtaskAdded == false){
            this.print("--in handleCNPauction not fully added!!");
            return;
        }

        this.cascadeTaskPair.put(taskSA.taskID, taskDA.taskID);
        this.sendMessage(this.generateOfferMessage(taskSA, offerVal, taskDA.ore, true));
    }




    public Message offerService(Pose fromPos, double nextStartTime) {
        ArrayList<Integer> receivers = this.getReceivers("DRAW");
        if (receivers.size() <= 0) return new Message();

        int DAtaskID = this.sendCNPmessage(nextStartTime, this.stringifyPose(fromPos), receivers);
        this.waitForAllOffersToCome(receivers.size(), DAtaskID);

        // offers in this.offers
        this.print("retriving best offer");
        Message bestOffer = this.handleOffers(DAtaskID); //extract best offer
        if ( bestOffer.isNull == false ){
            this.print("found good offer");
            receivers.removeIf(i -> i==bestOffer.sender);
        }
        if (receivers.size() > 0){
            Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(DAtaskID));
            this.sendMessage(declineMessage);
        }
        synchronized(this.offers){ this.offers.removeIf(i -> this.getMessageTaskID(i) == DAtaskID); }

        return bestOffer;
    }

    public Message handleOffersCascade(int taskID) {
        return null;
    }


    @Override
    protected void handleAccept(int taskID, Message m){
        Task taskDA;
        boolean eventAdded;

        synchronized(this.timeSchedule){
            taskDA = this.timeSchedule.getEvent(this.cascadeTaskPair.get(taskID));
            eventAdded = this.timeSchedule.setEventActive(taskDA.taskID);
            eventAdded = eventAdded ? this.timeSchedule.setEventActive(taskID) : false;
            this.print("accept-msg, taskID-->"+taskID+"\twith robot-->"+m.sender+"\ttask added-->"+eventAdded);
            this.timeSchedule.printSchedule(this.COLOR);
        }

        if ( eventAdded == false){
            synchronized(this.timeSchedule){
                this.timeSchedule.abortEvent(taskID);
                this.timeSchedule.abortEvent(taskDA.taskID);
            }
            this.sendMessage(new Message(this.robotID, m.sender, "inform", taskID+this.separator+"abort"));
            this.sendMessage(new Message(this.robotID, taskDA.partner, "inform", taskDA.taskID+this.separator+"abort"));

        } else {
            this.sendMessage(new Message(this.robotID, taskDA.partner, "accept", taskDA.taskID+""));
        }
    }

    @Override
    protected void handleDecline(int taskID, Message m){
        this.print("in handleDecline");
        Task taskDA = null;
        
        synchronized(this.timeSchedule){
            Integer DAtaskID = this.cascadeTaskPair.get(taskID);
            if ( DAtaskID != null ) taskDA = this.timeSchedule.getEvent(DAtaskID);
            if ( taskDA != null ) this.timeSchedule.removeEvent(taskDA.taskID);
            this.timeSchedule.removeEvent(taskID);
        }
        if ( taskDA != null ) this.sendMessage(new Message(this.robotID, taskDA.partner, "decline", taskDA.taskID+""));
        this.cascadeTaskPair.remove(taskID);
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
    protected int calculateOffer(Task taskSA, Task taskDA, Message m){
        if ( taskSA.pathDist < 2.0 ) return 0;

        // ore eval [1000, 0]
        int oreEval = Math.abs(taskSA.ore) > this.capacity-0.1 ? 1000 : (int)this.linearDecreasingComparingFunc(Math.abs(taskSA.ore), this.capacity, this.capacity, 500.0);
        
        // dist evaluation [500, 0]
        int distEval = (int)this.concaveDecreasingFunc(taskDA.pathDist, 500.0, 200.0); // [500, 0]
        distEval += (int)this.concaveDecreasingFunc(taskSA.pathDist, 500.0, 200.0); // [500, 0]
        // time bonus [1000, 0]
        double cnpEndTime = Double.parseDouble(this.parseMessage(m, "startTime")[0]);
        int timeEval = (int)this.linearDecreasingComparingFunc(taskSA.endTime, cnpEndTime, 60.0, 1000.0);  

        //TODO how long we have to sleep is part of equation
        
        //TODO add eval for using middle space of map. using middle space is BAD

        this.print("with robot-->"+m.sender +"\t dist-->"+ String.format("%.2f",taskSA.pathDist+taskDA.pathDist) 
                        +"\tdistance eval-->"+distEval
                        +"\t cnp endTime-->"+String.format("%.2f",cnpEndTime) 
                        +"\t timeDiff-->"+String.format("%.2f",Math.abs(cnpEndTime - taskSA.endTime )) 
                        +"\ttime eval-->"+timeEval);
        return oreEval + distEval + timeEval;
    }   
}