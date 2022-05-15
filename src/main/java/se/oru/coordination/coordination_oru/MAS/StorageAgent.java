package se.oru.coordination.coordination_oru.MAS;
import java.beans.EventHandler;
import java.util.ArrayList;
import java.util.HashMap;

import com.vividsolutions.jts.io.WKBConstants;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

public class StorageAgent extends BidderAgent{

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
        this.DIST_WEIGHT = mapInfo.getWeights(robotID)[0];
        this.ORE_WEIGHT = mapInfo.getWeights(robotID)[1];
        this.CONGESTION_WEIGHT = mapInfo.getWeights(robotID)[2];

        this.agentVelocity = mapInfo.getVelocity(2);
        this.TTAcapacity = mapInfo.getCapacity(4);
        this.TTAagentSpeed = mapInfo.getVelocity(4);

        this.timeSchedule = new TimeScheduleNew(oreState, this.initialPose, this.capacity, this.amount);
        this.clockStartTime = startTime;

        //this.occupancyPadding = 7.0;
        this.occupancyPadding = (25.0 / this.agentVelocity) * 1.7;

        // settings
        this.LOAD_DUMP_TIME = 0.0;//15.0 * 5.6 / this.agentVelocity;
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
    }

    /**
     * Stochastically update capacity
     */
    protected void changeCapacity() {
        // Sleep for 2 minutes
        this.sleep(1000 * 120);

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

    @Override
    protected void handleCNPauction(Message m){
        int agentType = (m.sender % 1000) / 100;

        Task cnpTask = agentType == 4 ? this.generateTTAtask(m) : this.generateTAtask(m); // Create Task
        if ( cnpTask == null ) return;
        
        double[] timeUsingResource = this.translateTAtaskTimesToOccupyTimes(cnpTask, this.occupancyPadding); 
        if ( !this.timeSchedule.isTaskPossible(cnpTask.taskID, timeUsingResource[0], timeUsingResource[1]) ) return;    // task doesnt fit in schedule
        
        int offerVal = agentType == 4 ? this.calculateOfferTTA(cnpTask, m) : this.calculateOfferTA(cnpTask, m); // Calculate offer
        this.print("--offerval-->"+(offerVal));
        if ( offerVal <= 0 ) return;
        
        this.sendMessage(this.generateOfferMessage(cnpTask, offerVal, Math.abs(cnpTask.ore))); // Send Offer
        cnpTask.startTime = timeUsingResource[0];
        cnpTask.endTime = timeUsingResource[1];
        this.timeSchedule.addEvent(cnpTask);    
    }

    private Task generateTTAtask(Message m){
        double availableOre;
        String[] mParts = this.parseMessage(m, "", true);
        Pose TruckPos = this.posefyString(mParts[2]);
        double startTime = Double.parseDouble(mParts[3]);
        Pose midWayPose = new Pose(TruckPos.getX(), 27.5, 0.0);
        double pathDist = this.basicPathDistEstimate(TruckPos, midWayPose) + this.basicPathDistEstimate(midWayPose, this.initialPose);
        double endTime = startTime + this.calculateDistTime(pathDist, this.TTAagentSpeed);
        synchronized(this.timeSchedule){ availableOre = this.timeSchedule.getOreStateAtTime(endTime); }
        availableOre = availableOre > this.TTAcapacity ? this.TTAcapacity : availableOre;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -availableOre, startTime, endTime, pathDist, TruckPos, this.initialPose);
    }

    private Task generateTAtask(Message m){
        double nextAvailableEndTime;
        String[] mParts = this.parseMessage(m, "", true);
        Pose TruckPos = this.posefyString(mParts[2]);
        double TAstart = Double.parseDouble(mParts[3]);
        double pathDist = this.basicPathDistEstimate(TruckPos, this.initialPose);
        double pathTime = this.calculateDistTime(pathDist, this.agentVelocity);

        synchronized(this.timeSchedule){ nextAvailableEndTime = this.timeSchedule.getSlotAfterTime(TAstart+pathTime-this.occupancyPadding+0.5, this.occupancyPadding*2.5, this.TAcapacity); }
        if ( nextAvailableEndTime == -1.0 ){
            this.print("next available time = -1.0. something wrong!");
            return null;
        } 
        double startTime = nextAvailableEndTime - pathTime;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, this.TAcapacity, startTime, nextAvailableEndTime, pathDist, TruckPos, this.initialPose);
    }

    protected int calculateOfferTTA(Task agentTask, Message autionMessage){
        if (agentTask.pathDist < 0.5) return 0;

        double oreLevel = this.timeSchedule.getOreStateAtTime(agentTask.endTime) - this.TTAcapacity;
        if ( oreLevel < 0.1*this.capacity ) return 0;

        int oreEval = (int)this.linearDecreasingComparingFunc(oreLevel, this.capacity, this.capacity, 1000);
       
        this.print("-- calculateOffer: offer->"+oreEval+" with agent->"+agentTask.partner);
        return oreEval;
    }

    protected int calculateOfferTA(Task agentTask, Message autionMessage){
        double oreLevelPercent = this.timeSchedule.getOreStateAtTime(agentTask.endTime) / this.capacity;

        int oreEval = (int)this.concaveDecreasingFunc(oreLevelPercent*100.0, 1000.0, 0.0, 100.0);

        int distEval = (int)this.concaveDecreasingFunc(agentTask.pathDist, 1000.0, 40.0, 300.0); 

        double requestStartTime = Double.parseDouble(this.parseMessage(autionMessage, "startTime")[0]); // startTime of cnp-msg is when auctioneer wants ore
        int timeEval = (int) (this.linearDecreasingComparingFunc(agentTask.startTime, requestStartTime, 45.0, 500.0)*this.TIME_WEIGHT);  
        // this.print("with robot-->"+m.sender +" dist-->"+ String.format("%.2f",t.pathDist) 
        //     +" distanceEval-->"+distEval
        //     +"\t nearTaskT-->"+String.format("%.2f",nearTaskT) 
        //     +" congEnval-->"+congestionEval
        //     +",  total eval->"+(oreEval + distEval + congestionEval));

        //this.print("--calcualteOFferTA: orelevel percent ->"+oreLevelPercent);
        return (int)(oreEval*this.ORE_WEIGHT) + (int)(distEval*this.DIST_WEIGHT) + (int)(timeEval*this.TIME_WEIGHT);
    } 

    @Override
    protected void handleInformDone(int taskID, Message m){
        ArrayList<Task> abortTasks;
        int agentType = (m.sender % 1000) / 100;
        double oreAmount = Math.abs( Double.parseDouble(this.parseMessage(m, "informInfo")[0]) );
        oreAmount = agentType == 2 ? oreAmount : -oreAmount;
        synchronized(this.timeSchedule){
            this.timeSchedule.setNewOreAmount(taskID, oreAmount);
            abortTasks = this.timeSchedule.fixBrokenSchedule();
        }
        for ( Task t : abortTasks ){
            this.sendMessage( new Message(this.robotID, t.partner, "inform", t.taskID + this.separator + "abort") );
        }
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

        this.print("in handleInformStatus");
        //this.timeSchedule.printSchedule(this.COLOR);

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
}
