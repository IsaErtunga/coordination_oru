package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Arrays;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;


public class TransportTruckAgent extends MobileAgent{
    //Control parameters

    protected Pose[] corners;

    // Begins at 4. Will iterate through SW, NW, NE, SE
    protected int cornerState = 8;
    protected Pose deliveryPos;
    public ArrayList<Message> missionList = new ArrayList<Message>();
    

    public TransportTruckAgent(int id){this.robotID = id;}   // for testing

    public TransportTruckAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
                        ReedsSheppCarPlanner mp, Pose startPos, Router router){}

    public TransportTruckAgent( int r_id, TrajectoryEnvelopeCoordinatorSimulation tec, NewMapData mapInfo, Router router,
                                long startTime, ReedsSheppCarPlanner mp, HashMap<String, PoseSteering[]> pathStorage){
        
        
        this.robotID = r_id;
        this.COLOR = "\033[1;94m";
        this.tec = tec;
        this.initialPose = mapInfo.getPose(r_id);
        this.clockStartTime = startTime;

        this.agentVelocity = mapInfo.getVelocity(4);
        this.setRobotSpeedAndAcc(this.agentVelocity, 20.0);
        this.rShape = mapInfo.getAgentSize(r_id);
        this.mp = mp;
        this.deliveryPos = mapInfo.getPose(-1);

        this.pStorage = pathStorage;
        this.capacity = mapInfo.getCapacity(r_id);

        this.LOAD_DUMP_TIME = 0.0;//15.0 * 5.6 / (this.agentVelocity*2);
        this.taskCap = 2;

        this.timeSchedule = new TimeScheduleNew(this.initialPose, this.capacity, mapInfo.getStartOre(r_id));

        this.print("constructor");
        this.print("loadDump time-->"+this.LOAD_DUMP_TIME);

        this.corners = mapInfo.getCorners();

        // enter network and broadcast our id to others.
        router.enterNetwork(this);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);
        
    }

     /**
     * 
     */
    public void start(){
        TransportTruckAgent This = this;

        this.addRobotToSimulation();

        Thread listenerThread = new Thread() {
            public void run() {
                This.listener();
            }
        };
        listenerThread.start();

        Thread stateThread = new Thread() {
            public void run() {
                This.initialState();
            }
        };
        stateThread.start();

        this.stateHandler();
    }

    /**
     * Updates which corner the TTA is on.
     */
    protected int incrementCornerState() {
        if (this.cornerState == 8) {
            this.cornerState = 1;
        } 
        else {
            this.cornerState = this.cornerState + 1;
        }
        this.print("CORNER STATE: "+ this.cornerState);
        //System.out.println("CORNER STATE: "+ this.cornerState);
        return this.cornerState;
    }

 

    /** //TODO looks good for now. will use timeSchedule.evaluateTimeSlot() in future
     * 
     * HandleOffers is called from a SA, to either accept the offer of a TA, or deny it.
     * @return the message that is the best offer
     */
    public Message handleOffers(int taskID) {
        Message bestOffer = new Message();
        int offerVal = 0;
        int SAOrderVAl = 0;
    
        for (Message m : this.offers) {
            // SCHEDULE: Extract startTime & endTime and see if it fits into schedule
            double taskStartTime = Double.parseDouble(parseMessage(m, "startTime")[0]);
            double endTime = Double.parseDouble(parseMessage(m, "endTime")[0]);

            boolean taskPossible;
            synchronized(this.timeSchedule){ taskPossible = this.timeSchedule.isTaskPossible(taskID, taskStartTime, endTime); }
  
            if(taskPossible) {
                String[] mParts = this.parseMessage( m, "", true); // sort out offer not part of current auction(taskID)

                if ( Integer.parseInt(mParts[0]) == taskID ){
                    int val = Integer.parseInt(mParts[1]);

                    if (val > offerVal){
                        offerVal = val;
                        bestOffer = new Message(m);
                    }
                    //int SAOrderNumber = m.sender - 5000;

                    // if (val > offerVal) {
                    //     if (val < offerVal * 1.1) {
                    //         // If only 10% larger or less
                    //         if (SAOrderNumber > SAOrderVAl) {
                    //             // Check precedence
                    //             offerVal = val;
                    //             SAOrderVAl = SAOrderNumber;
                    //             bestOffer = new Message(m);
                    //         }
                    //     }
                    //     else {
                    //         offerVal = val;
                    //         SAOrderVAl = SAOrderNumber;
                    //         bestOffer = new Message(m);
                    //     }
                    // }
                    // else {
                    //     if (val > offerVal * 0.9) {
                    //         // If only 10% smaller or less
                    //         if (SAOrderNumber > SAOrderVAl) {
                    //             // Check precedence
                    //             offerVal = val;
                    //             SAOrderVAl = SAOrderNumber;
                    //             bestOffer = new Message(m);
                    //         }
                    //     }
                    // }
                }
            }
        }
        return bestOffer;
    }

    @Override
    public Mission createMission(Task task, Pose prevToPose) {
        Pose[] toPose = this.navigateCorrectly(task, task.ore > 0.0);
        PoseSteering[] path = this.getPath(this.pStorage, this.mp, prevToPose, toPose);
        return new Mission( this.robotID, path );
    }

    /**
     * Function that creates a task for the TTA to deliver ore out of mine. 
     * @return Task deliverTask
     */
    protected Task createDeliverTask() {
        Pose robotPos;
        synchronized(this.timeSchedule) {
            robotPos = this.timeSchedule.getNextPose();
        }
        //Pose deliveryPos = new Pose(245.0, 105.0, Math.PI);	
        PoseSteering[] path = this.getPath(this.pStorage, this.mp, robotPos, this.deliveryPos);
        double pathDist = this.calculatePathDist(path);
        double pathTime = this.calculateDistTime(pathDist, this.agentVelocity);
        double taskStartTime = this.getNextTime();
        double endTime = taskStartTime + pathTime;
        Task deliverTask = new Task(this.tID(), -1, true, -this.capacity, taskStartTime, endTime, pathDist, robotPos, this.deliveryPos);
        return deliverTask;
    }

    /**
     * 
     * @param task
     * @param toSA
     * @return
     */
    public Pose[] navigateCorrectly(Task task, boolean toSA) {
        ArrayList<Pose> localCorners = new ArrayList<Pose>();

        int lastCorner = toSA ? 2 : 6; // if pickup: 2

        while (this.cornerState != lastCorner) {
            localCorners.add(this.corners[this.incrementCornerState()-1]);
        }
        localCorners.add(task.toPose);
        
        return localCorners.toArray(new Pose[0]);
    }

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportTruckAgent} calling this
     */
    public Message offerService(double taskStartTime) {
        // Get correct receivers
        ArrayList<Integer> receivers = this.getReceivers("STORAGE");

        if (receivers.size() <= 0) return new Message();

        this.offers.clear();
        
        Pose nextPose;
        synchronized(this.timeSchedule){
            nextPose = this.timeSchedule.getNextPose();
        }

        String startPos = this.stringifyPose(nextPose);
        int taskID = this.sendCNPmessage(taskStartTime, startPos, receivers);


        this.waitForAllOffersToCome(receivers.size(), taskID);
    
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

    protected void initialState() {
        double oreLevelThreshold = 1.0;
        while (true) {
            this.sleep(500);

            double lastOreState;
            synchronized(this.timeSchedule){
                if ( this.timeSchedule.getSize() > this.taskCap ) continue;
                lastOreState = this.timeSchedule.getLastOreState();
            }
            
            if (lastOreState <= oreLevelThreshold) { 
                // book task to get ore
                Message bestOffer = this.offerService(this.getNextTime());

                if (bestOffer.isNull) continue;
                
                Task task = this.generateTaskFromOffer(bestOffer);

                boolean taskAdded;
                synchronized(this.timeSchedule){ taskAdded = this.timeSchedule.addEvent(task); }
                if ( taskAdded != true ){ // if false then task no longer possible, send abort msg to task partner
                    this.print("TASK ABORTED");
                    this.sendMessage(new Message(this.robotID, task.partner, "inform", Integer.toString(task.taskID)+this.separator+"abort"));
                }
                // this.print("initialState -- task added");
                // this.timeSchedule.printSchedule(this.COLOR);
            }

            else {
                this.print("initialState -- in else");
                //this.timeSchedule.printSchedule(this.COLOR);
                // Deliver ore 
                Task deliverTask = createDeliverTask();
                boolean taskAdded;
                synchronized(this.timeSchedule){ taskAdded = this.timeSchedule.addEvent(deliverTask); }

                if ( taskAdded != true ){ // if false then task no longer possible, send abort msg to task partner
                    this.print("TASK ABORTED");
                    this.sendMessage(new Message(this.robotID, deliverTask.partner, "inform", deliverTask.taskID+this.separator+"abort"));
                }
                // this.timeSchedule.printSchedule("");
            }

            
        }
    }
}