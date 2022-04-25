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

    protected HashMap<String, PoseSteering[]> pStorage;

    // Begins at 4. Will iterate through SW, NW, NE, SE
    protected int cornerState = 4;

    public ArrayList<Message> missionList = new ArrayList<Message>();
    

    public TransportTruckAgent(int id){this.robotID = id;}   // for testing

    public TransportTruckAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
                        ReedsSheppCarPlanner mp, Pose startPos, Router router){}

    public TransportTruckAgent( int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,ReedsSheppCarPlanner mp,
                                Pose startPos, Router router, long startTime, HashMap<String, PoseSteering[]> pathStorage){
            
                            System.out.println("#######################");
                            System.out.println(r_id +" -- constructor");
        
        this.robotID = r_id;
        this.tec = tec;
        this.mp = mp;
        this.initialPose = startPos;
        this.clockStartTime = startTime;
        this.pStorage = pathStorage;
        this.COLOR = "\033[1;94m";
        this.capacity = 40.0;

        // settings 
        this.TASK_EXECUTION_PERIOD_MS = 200;
        this.taskCap = 4;
        this.robotAcceleration = 10.0;
        this.robotSpeed = 20.0;

        this.timeSchedule = new TimeScheduleNew(startPos, this.capacity, 0.0);

        this.setRobotSize(5.0, 3.7);

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

        this.taskExecutionThread();
    }

    /**
     * Updates which corner the TTA is on.
     */
    protected int incrementCornerState() {
        if (this.cornerState == 4) {
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
        Pose deliveryPos = new Pose(245.0, 105.0, Math.PI);	
        PoseSteering[] path = this.getPath(this.pStorage, this.mp, robotPos, deliveryPos);
        double pathDist = this.calculatePathDist(path);
        double pathTime = this.calculateDistTime(pathDist);
        double taskStartTime = this.getNextTime();
        double endTime = taskStartTime + pathTime;
        Task deliverTask = new Task(this.tID(), -1, true, -this.capacity, taskStartTime, endTime, endTime, robotPos, deliveryPos);
        return deliverTask;
    }

    /**
     * 
     * @param task
     * @param toSA
     * @return
     */
    public Pose[] navigateCorrectly(Task task, boolean toSA) {
        ArrayList<Pose> corners = new ArrayList<Pose>();
        int lastCorner;
        if (toSA) {
            // IF PICKUP ORE
            lastCorner = 1;
        } else {
            lastCorner = 3;
        }

        while (this.cornerState != lastCorner) {
            corners.add(task.corners[this.incrementCornerState()-1]);
        }
        corners.add(task.toPose);
        
        return corners.toArray(new Pose[0]);
    }

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportTruckAgent} calling this
     */
    public Message offerService(double taskStartTime) {
        this.print("in offerService");
        // Get correct receivers
        ArrayList<Integer> receivers = this.getReceivers("STORAGE");

        if (receivers.size() <= 0) return new Message();
        this.print("receivers > 0");

        this.offers.clear();
        
        Pose nextPose;
        int scheduleSize;
        synchronized(this.timeSchedule){
            nextPose = this.timeSchedule.getNextPose();
            scheduleSize = this.timeSchedule.getSize();
        }
        if ( scheduleSize > this.taskCap ) return new Message();
        this.print("taskCap < scSize");


        String startPos = this.stringifyPose(nextPose);
        int taskID = this.sendCNPmessage(taskStartTime, startPos, receivers);


        double time = this.waitForAllOffersToCome(receivers.size(), taskID);
        this.print("time waited for offers-->"+time);
    
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
                this.print("initialState -- lastOreState < threashold");
                this.timeSchedule.printSchedule(this.COLOR);
            }

            else {
                this.print("initialState -- in else");
                this.timeSchedule.printSchedule(this.COLOR);
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