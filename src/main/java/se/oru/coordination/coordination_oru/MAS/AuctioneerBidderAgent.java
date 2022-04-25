package se.oru.coordination.coordination_oru.MAS;
import org.metacsp.multi.spatioTemporal.paths.Pose;

public class AuctioneerBidderAgent extends AuctioneerAgent{
    

    // ===================== BIDDER FUNCTIONS BELOW ===================
    // ================================================================
    // ================================================================

    /**
     * Basic offer calc func. this needs to be overwritten in final agent object
     * @param agentTask the task generated from the auction
     * @param autionMessage the auction message
     * @return an int value that is the offer 
     */
    protected int calculateOffer(Task agentTask, Message autionMessage){
        // ore eval [1000, 0]
        int oreEval = (int)Math.abs(agentTask.ore);
        // dist evaluation [1000, 0]
        int distEval = (int)agentTask.fromPose.distanceTo(agentTask.toPose);
        // time bonus [100, 0]
        int timeEval = 0;
        return oreEval + distEval + timeEval;
    }

    /**
     * When receiving a cnp-message, this function will create a task from that message.
     * Only used in {@link handleService}.
     * @param m the message from the auctioneer
     * @param ore the amount of ore the task is about
     * @return a Task with attributes extracted from m
     */
    protected Task generateTaskFromAuction(Message m, Pose ourPose, double ore){
        double TIME_ADD = 4.0;
        String[] mParts = this.parseMessage(m, "", true);

        Pose auctioneerPose = this.posefyString(mParts[2]);
        double pathDist = ourPose.distanceTo(auctioneerPose);
        double pathTime = this.calculateDistTime(pathDist) + TIME_ADD;
        double taskStartTime = Double.parseDouble(mParts[3]);
        double endTime = taskStartTime + pathTime;

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -ore, taskStartTime, endTime, pathTime, auctioneerPose, ourPose);
    }

    /**
     * Used to generate a response message from a task. Called from {@link handleService}
     * after creating a task with {@link createTaskFromServiceOffer}.
     * @param t a Task that is unactive = t.isActive = false
     * @param offer an int that is the calculated evaluation of the service related to t
     * @param ore a double representing the ore amount the task handels
     * @return returns a Message with attributes extracted from the parameters
     */
    protected Message generateOfferMessage(Task task, int offerVal, double oreChange){
        String s = this.separator;
        String TAposStr = this.stringifyPose(task.fromPose);
        String DAposStr = this.stringifyPose(task.toPose);
        String body = task.taskID +s+ offerVal +s+ TAposStr +s+ DAposStr +s+ task.startTime +s+ task.endTime +s+ Math.abs(oreChange);

        return new Message(this.robotID, task.partner, "offer", body);
    }

    /** handleService is called from within a TA, when a TA did a {@link offerService}
     * @param m the message with the service
     * @param robotID the robotID of this object
     * @return true if we send offer = we expect resp.
     */
    protected boolean handleService(Message m, double availableOre, Pose agentPose) { 

        Task auctionTask = this.generateTaskFromAuction(m, agentPose, availableOre);

        boolean taskPossible;
        synchronized(this.timeSchedule){ taskPossible = this.timeSchedule.isTaskPossible(auctionTask); }
        if ( !taskPossible ) return false;    // task doesnt fit in schedule

        int offerVal = this.calculateOffer(auctionTask, m);
        
        if ( offerVal <= 0 ) return false;

        synchronized(this.timeSchedule){ this.timeSchedule.addEvent(auctionTask); }

        this.sendMessage(this.generateOfferMessage(auctionTask, offerVal, availableOre));

        return true;
    }
    
    /**
     * example of how handleCNPauction could look using taskCap, and checking if available ore > 0
     */
    @Override
    protected void handleCNPauction(Message m){
        double availableOre;
        Pose pose;
        int scheduleSize;

        synchronized(this.timeSchedule){
            scheduleSize = this.timeSchedule.getSize();
            availableOre = this.timeSchedule.getLastOreState();
            pose = this.timeSchedule.getNextPose();
        }
        if ( availableOre <= 0.01 || scheduleSize >= this.taskCap) return;

        this.handleService(m, availableOre, pose);
    }
}
