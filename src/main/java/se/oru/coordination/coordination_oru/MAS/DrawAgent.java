/**
 * 
 */
package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Arrays;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.MAS.Router;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.Mission;

public class DrawAgent extends BidderAgent{
    // control parameters

    private double finalXPos;
    protected double initalXPos;
    protected double currentOreAmount;

    protected boolean shiftLeft;

    public DrawAgent(int robotID, Router router, double capacity, Pose pos, ReedsSheppCarPlanner mp){}

    public DrawAgent(   int robotID, Router router, double capacity, Pose pos, ReedsSheppCarPlanner mp,
                        long startTime, boolean shiftLeft){

        this.robotID = robotID; // drawID >10'000
        this.capacity = capacity;
        this.initialOreAmount = capacity;
        this.currentOreAmount = capacity;
        this.mp = mp;
        this.router = router;
        this.initialPose = pos;
        this.initalXPos = pos.getX();
        this.COLOR = "\033[0;36m";

        this.timeSchedule = new TimeScheduleNew(pos, capacity, this.initialOreAmount);
        this.clockStartTime = startTime;

        router.enterNetwork(this.robotID, this.inbox, this.outbox);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);

        this.shiftLeft = shiftLeft;
        if (shiftLeft) {
            this.finalXPos = this.initalXPos - 32.0;
        } else {
            this.finalXPos = this.initalXPos + 32.0;
        }

    }

    public void takeOre(double oreChange){
        oreChange = oreChange < 0.0 ? oreChange : -oreChange; // make sure sign is negative
        this.currentOreAmount += oreChange;

        double x;
        if (this.shiftLeft) x = this.finalXPos + 32.0 * (this.currentOreAmount / this.capacity);
        else x = this.finalXPos - 32.0 * (this.currentOreAmount / this.capacity);

        this.initialPose = new Pose( x, initialPose.getY(), initialPose.getYaw() );
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
        if ( abortTasks.size() > 0 ){
            //this.print("---schedule---AFTER");
            this.timeSchedule.printSchedule(this.COLOR);
        }
    }

    @Override
    protected void handleCNPauction(Message m){
        double START_TIME_PADDING = 5.0;
        double startTime = Double.parseDouble( this.parseMessage(m, "startTime")[0] ) +START_TIME_PADDING;

        double availableOre = this.timeSchedule.getOreStateAtTime(startTime);
        Pose agentPose = this.calculateFuturePos(startTime);

        if (availableOre <= 0.01){ //if we dont have ore dont act 
            this.goOffline = true;
            this.print("no ore");
            return;   
        } 
        else availableOre = availableOre >= 15.0 ? 15.0 : availableOre;
        this.handleService(m, availableOre, agentPose);
    }

    @Override
    protected void handleInformDone(int taskID, Message m){
        double oreChange = Double.parseDouble(this.parseMessage(m, "informInfo")[0]);
        this.timeSchedule.removeEvent(taskID);
        this.takeOre(oreChange);
    };

    @Override
    protected void handleInformStatus(Message m){
        //this.print("in handleStatusMessage");
        String updateSep = "::";
        String pairSep = ":";

        String informInfo = (this.parseMessage(m, "informInfo")[0]);
        //this.print(informInfo);

        String[] newTimes = informInfo.split(updateSep);
        for ( int i=0; i<newTimes.length; i++ ){
            //this.print("\tnewTimes[i]-->"+ newTimes[i]);
            String[] updatePair = newTimes[i].split(pairSep);
            //this.print("\tupdatePair successfully split");

            int taskID = Integer.parseInt( updatePair[0] );
            double newEndTime = Double.parseDouble( updatePair[1] );
            //this.print("\t"+ "taskID-->"+taskID+"newEndTime-->"+newEndTime);

            Task taskToAbort = null;
            synchronized(this.timeSchedule) { taskToAbort = this.timeSchedule.updateTaskEndTimeIfPossible(taskID, newEndTime); }
            if ( taskToAbort != null ){
                this.sendMessage(new Message(this.robotID, taskToAbort.partner, "inform", taskToAbort.taskID+this.separator+"abort"));
                this.print("CONFLICT! sending ABORT msg. taskID-->"+taskID+"\twith-->"+m.sender );
            } else {
                this.print("updated without conflict-->"+taskID +"\twith-->"+ m.sender);
            }
        }
    };

    /**
     * DA responds to TA with offer that is calculated in this function. 
     * SCHEDULE:
     * - Will receive a time from TA of when it can come and fetch ore. 
     *
    public boolean handleService(Message m){ 
        double startTime = Double.parseDouble( this.parseMessage(m, "startTime")[0] );
        double availabeOre = this.timeSchedule.getOreStateAtTime(startTime+5.0);
        if (availabeOre <= 0.0){ //if we dont have ore dont act 
            this.print("no ore");
            return false;   
        } 
        else availabeOre = availabeOre >= 15.0 ? 15.0 : availabeOre; // only give what ore we have available

        Task DAtask = createTaskFromServiceOffer(m, availabeOre);

        if ( !this.timeSchedule.isTaskPossible(DAtask) ) return false;    // task doesnt fit in schedule
        int offerVal = this.calculateOffer(DAtask, m);

        if ( offerVal <= 0 ) return false;
        if (! this.timeSchedule.addEvent(DAtask) ) return false;
        
        // this.print("--- schedule ---");
        // this.timeSchedule.printSchedule(this.COLOR);

        this.sendMessage(this.createOfferMsgFromTask(DAtask, offerVal, availabeOre));
        
        // ArrayList<Task> abortTasks = this.timeSchedule.fixBrokenSchedule();
        // for (Task t : abortTasks){
        //     this.print("CONFLICT! sending ABORT msg. taskID-->"+t.taskID+"\twith-->"+t.partner);
        //     this.sendMessage(new Message(this.robotID, t.partner, "inform", t.taskID+this.separator+"abort"));
        // }
        return true;
    }

    /**
     * Function that determines if DrawAgent moves right or left
     * And how much.
     * @param time
     * @return
     */
    protected Pose calculateFuturePos(double time){
        double oreAtTime = this.timeSchedule.getOreStateAtTime(time);
        double x;
        if (this.shiftLeft) x = this.finalXPos + 32.0 * (oreAtTime / this.capacity);
        else x = this.finalXPos - 32.0 * (oreAtTime / this.capacity);
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
        int oreEval = Math.abs(t.ore) > 14.9 ? 1000 : (int)this.linearDecreasingComparingFunc(Math.abs(t.ore), 15.0, 15.0, 500.0);
        
        // dist evaluation [1000, 0]
        int distEval = (int)this.concaveDecreasingFunc(t.fromPose.distanceTo(t.toPose), 1000.0, 120.0); // [1000, 0]

        // time bonus [100, 0]
        double cnpStartTime = Double.parseDouble(this.parseMessage(m, "startTime")[0]);
        double upperTimeDiff = cnpStartTime <= 0.0 ? 60.0 : 30.0;
        int timeEval = (int)this.linearDecreasingComparingFunc(t.startTime, cnpStartTime, upperTimeDiff, 100.0);

        /*
        this.print("with robot-->"+m.sender +"\t dist-->"+ String.format("%.2f",t.pathDist) 
            +"\tdistance eval-->"+distEval
            +"\t cnp starttime-->"+String.format("%.2f",cnpStartTime) 
            +"\t timeDiff-->"+String.format("%.2f",Math.abs(cnpStartTime - t.startTime )) 
            +"\ttime eval-->"+timeEval);
        */
        return oreEval + distEval + timeEval;
    }
    
}
