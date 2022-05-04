/**
 * 
 */
package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

public class DrawAgent extends BidderAgent{
    // control parameters

    private double finalXPos;
    protected double initalXPos;

    public DrawAgent(int robotID, Router router, double capacity, Pose pos, ReedsSheppCarPlanner mp){}

    public DrawAgent(   int robotID, Router router, double capacity, Pose pos, ReedsSheppCarPlanner mp,
                        long startTime){} // old, not used anymore
    public DrawAgent( int robotID, Router router, NewMapData mapInfo, long startTime, ReedsSheppCarPlanner mp){

        this.robotID = robotID;
        this.COLOR = "\033[0;36m";

        this.capacity = mapInfo.getCapacity(robotID);
        this.amount= mapInfo.getStartOre(robotID);

        this.mp = mp;
        this.agentVelocity = mapInfo.getVelocity(2); // 2 is for TA beacuse it only interacts with TA's
        this.TAcapacity = mapInfo.getCapacity(2);
        this.initialPose = mapInfo.getPose(robotID);
        this.initalXPos = this.initialPose.getX();
        this.finalXPos = this.initalXPos - 70.0;

        this.timeSchedule = new TimeScheduleNew(this.initialPose, this.capacity, this.amount);
        this.clockStartTime = startTime;
        this.occupancyPadding = 4.0;
        this.LOAD_DUMP_TIME = 0.0;//15.0 * 5.6 / this.agentVelocity;

        this.print("initiated");
        this.print("loadDump time-->"+this.LOAD_DUMP_TIME);
        this.router = router;
        router.enterNetwork(this.robotID, this.inbox, this.outbox);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);
        
    }

    @Override
    protected void handleAccept(int taskID, Message m){
        this.print("---schedule---BEFORE");
        this.timeSchedule.printSchedule(this.COLOR);
        boolean eventAdded;
        synchronized(this.timeSchedule){ eventAdded = this.timeSchedule.setEventActive(taskID, true); }
        //this.print("accept-msg, taskID-->"+taskID+"\twith robot-->"+m.sender+"\ttask added-->"+eventAdded);
        if ( eventAdded == false ){
            //this.print("accept received but not successfully added. sending abort msg");
            this.sendMessage(new Message(this.robotID, m.sender, "inform", taskID+this.separator+"abort"));
        }

        ArrayList<Task> abortTasks = this.timeSchedule.fixBrokenSchedule();
        for (Task t : abortTasks){
            //this.print("CONFLICT from fixBrokenSchedule, sending ABORT msg. taskID-->"+t.taskID+"\twith-->"+t.partner);
            this.sendMessage(new Message(this.robotID, t.partner, "inform", t.taskID+this.separator+"abort"));
        }
    }

    @Override
    protected void handleCNPauction(Message m){
        this.print("cnp msg received");
        double startTime = Double.parseDouble( this.parseMessage(m, "startTime")[0] );

        double availableOre = this.timeSchedule.getOreStateAtTime(startTime);
        Pose agentPose = this.calculateFuturePos(startTime);

        if ( availableOre <= 0.01 ) return;
        else availableOre = availableOre >= this.TAcapacity ? this.TAcapacity : availableOre;

        Task auctionTask = this.generateTaskFromAuction(m, agentPose, availableOre);

        // ========= EXPERIMENTAL =========
        //double padding = this.calculateDistTime(agentPose.distanceTo(this.initialPose), this.agentVelocity)*this.paddingFactor;
        double padding = (agentPose.distanceTo(this.initialPose) / this.agentVelocity) + this.occupancyPadding/2;
        double[] timeUsingResource = this.translateTAtaskTimesToOccupyTimes(auctionTask, padding);
        boolean taskPossible = this.timeSchedule.isTaskPossible(auctionTask.taskID, timeUsingResource[0], timeUsingResource[1]);         
        if ( taskPossible == false ) return;    // task doesnt fit in schedule

        int offerVal = this.calculateOffer(auctionTask, m);
        if ( offerVal <= 0 ) return;

        this.sendMessage(this.generateOfferMessage(auctionTask, offerVal, availableOre));
        auctionTask.startTime = timeUsingResource[0];
        auctionTask.endTime = timeUsingResource[1];
        this.timeSchedule.addEvent(auctionTask);
        // ================================
    }

    @Override
    protected void handleInformDone(int taskID, Message m){
        synchronized(this.timeSchedule){ this.amount = this.timeSchedule.markEventDone(taskID); }
        this.print("currentOre -->"+this.amount);
    }

    @Override
    protected void handleInformStatus(Message m){
        String updateSep = "::";
        String pairSep = ":";

        String informInfo = (this.parseMessage(m, "informInfo")[0]);
        String[] newTimes = informInfo.split(updateSep);
        for ( int i=0; i<newTimes.length; i++ ){
            String[] updatePair = newTimes[i].split(pairSep);
            Task task = this.timeSchedule.getEvent(Integer.parseInt( updatePair[0] )); //altered
            //double padding = task.toPose.distanceTo(this.initialPose)*this.paddingFactor; // added
            double padding = (task.toPose.distanceTo(this.initialPose) / this.agentVelocity) + this.occupancyPadding/2;
            double newEndTime = Double.parseDouble( updatePair[1] ) + padding;  // altered
    
            Task taskToAbort = null;
            synchronized(this.timeSchedule) { taskToAbort = this.timeSchedule.updateTaskEndTimeIfPossible(task.taskID, newEndTime); }
            if ( taskToAbort != null ){
                this.sendMessage(new Message(this.robotID, taskToAbort.partner, "inform", taskToAbort.taskID+this.separator+"abort"));
                this.print("CONFLICT! sending ABORT msg. taskID-->"+taskToAbort.taskID+"\twith-->"+taskToAbort.partner );
            } else {
                this.print("updated without conflict-->"+task.taskID +"\twith-->"+ m.sender);
            }
        }
    }

    /**
     * Function that determines if DrawAgent moves right or left
     * And how much.
     * @param time
     * @return
     */
    protected Pose calculateFuturePos(double time){
        double oreAtTime = this.timeSchedule.getOreStateAtTime(time);
        double x = this.finalXPos + 70.0 * (oreAtTime / this.capacity);
        return new Pose( x, this.initialPose.getY(), this.initialPose.getYaw() );
    }

    /**
     * this function evaluates a task and calculates how good this task fits for this agent.
     * If this function determins that we dont want to participate in the auction this function
     * will return 0.
     * @param t the task to be evaluated how well it fits for this agent
     * @return an int with the value of this task.
     */
    @Override
    protected int calculateOffer(Task t, Message m){
        if (t.pathDist <= 2.0) return 0;

        // ore eval [1000, 0]
        int oreEval = Math.abs(t.ore) > this.TAcapacity-0.1 ? 1000 : (int)this.linearDecreasingComparingFunc(Math.abs(t.ore), this.TAcapacity, this.TAcapacity, 500.0);
        
        // dist evaluation [1000, 0]
        int distEval = (int)this.concaveDecreasingFunc(t.pathDist, 1000.0, 350.0); // [1000, 0]

        // time bonus [100, 0]
        double cnpStartTime = Double.parseDouble(this.parseMessage(m, "startTime")[0]);
        //int timeEval = (int)this.linearDecreasingComparingFunc(t.startTime, cnpStartTime, 60.0, 100.0);
        int timeEval = 0;

        // congestion eval [500, 0]
        double nearTaskT = this.timeSchedule.evaluateEventSlot(t.startTime, t.endTime, t.partner);
        nearTaskT = nearTaskT == -1.0 ? 30.0 : nearTaskT < 30.0 ? nearTaskT : 30.0;
        int congestionEval = (int)this.linearIncreasingComparingFunc(nearTaskT, 0.0, 30.0, 500.0);


        this.print("with robot-->"+m.sender +" dist-->"+ String.format("%.2f",t.pathDist) 
            +" distanceEval-->"+distEval
            +"\t cnp starttime-->"+String.format("%.2f",cnpStartTime) 
            +" timeDiff-->"+String.format("%.2f",Math.abs(cnpStartTime - t.endTime )) 
            +"\t nearTaskT-->"+String.format("%.2f",nearTaskT) 
            +" congEnval-->"+congestionEval);
        
        return oreEval + distEval + timeEval + congestionEval;
    }
    
}
