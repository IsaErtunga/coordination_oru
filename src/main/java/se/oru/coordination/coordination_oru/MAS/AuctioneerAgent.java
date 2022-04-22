package se.oru.coordination.coordination_oru.MAS;
import java.util.ArrayList;
import org.metacsp.multi.spatioTemporal.paths.Pose;

public class AuctioneerAgent extends BasicAgent{
    protected int STATUS_FREQ_MS = 200;
    protected int ORE_LEVEL_LOWER;
    protected int ORE_LEVEL_UPPER;
    protected int TIME_WAITING_FOR_OFFERS;


    protected void statusThread(){
        while ( true ){
            this.sleep(this.STATUS_FREQ_MS);

            double oreLevel;
            int scheduleSize;
            double nextStartTime;
            synchronized(this.timeSchedule){
                oreLevel = this.timeSchedule.getLastOreState();
                scheduleSize = this.timeSchedule.getSize();
                nextStartTime = this.timeSchedule.getNextStartTime();
            }

            if ((oreLevel < this.ORE_LEVEL_UPPER || oreLevel > this.ORE_LEVEL_LOWER) && scheduleSize < this.taskCap) { // plan future tasks
                Message bestOffer = this.holdAuction(nextStartTime);

                if ( bestOffer.isNull == true ) continue;

              
                Task task = this.generateTaskFromOffer(bestOffer);

                boolean taskAdded;
                synchronized(this.timeSchedule){ taskAdded = this.timeSchedule.addEvent(task); }
                if (taskAdded == false) this.sendMessage(new Message(this.robotID, task.partner, "inform", task.taskID+this.separator+"abort"));
                
            }
            else {
                this.print("not doing anything atm...");
            }
        }
    }

    protected Message holdAuction(double startTime){
        Pose poseAtStartTime = null;
        ArrayList<Integer> receivers = this.getReceivers("TRANSPORT");
        if ( receivers.size() <= 0) return new Message();

        return this.offerService(poseAtStartTime,  startTime, receivers);
    }

    protected Task generateTaskFromOffer(Message offerMsg){
        String[] msgParts = parseMessage(offerMsg, "", true);
        // replace intexes
        return new Task(Integer.parseInt(msgParts[0]), offerMsg.sender, true, Double.parseDouble(msgParts[6]), Double.parseDouble(msgParts[4]),
                        Double.parseDouble(msgParts[5]), this.posefyString(msgParts[2]), this.posefyString(msgParts[3]));
    }

    /** This function is locking. will wait for offers from all recipients to arrive OR return on the time delay.
     * 
     * @param nrOfReceivers an int with the number of recipients of the cnp message sent. 
     * @return a double with the time it took before exiting this function.
     */
    protected double waitForAllOffersToCome(int nrOfReceivers, int taskID){
        double before = this.getTime();
        ArrayList<Message> offerListCopy;

        while ( this.getTime() - before <= this.TIME_WAITING_FOR_OFFERS){
            int offersReceived = 0;
            synchronized(this.offers){ offerListCopy = new ArrayList<Message>(this.offers); }

            for ( Message offer : offerListCopy ){
                if ( this.getMessageTaskID(offer) == taskID ) offersReceived++;
            }
            
            if ( offersReceived >= nrOfReceivers ) return (this.getTime() - before);

            this.sleep(50);
        }
        return (this.getTime() - before);
    }

/** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportAgent} calling this
     */
    public Message offerService(Pose auctionStartPose, double startTime, ArrayList<Integer> receivers){
        this.offers.clear();
        int taskID = this.sendCNPmessage(startTime, this.stringifyPose(auctionStartPose), receivers);

        this.waitForAllOffersToCome( receivers.size(), taskID );  //locking function. wait for receivers
        Message bestOffer = this.handleOffers(taskID); //extract best offer

        if ( bestOffer.isNull == true ){
            this.print("no good offer received");
            Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(taskID));
            this.sendMessage(declineMessage);
            return bestOffer;
        }

        Message acceptMessage = new Message(robotID, bestOffer.sender, "accept", Integer.toString(taskID) );
        this.sendMessage(acceptMessage);

        receivers.removeIf(i -> i==bestOffer.sender);
        if (receivers.size() > 0){
            Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(taskID));
            this.sendMessage(declineMessage);
        }
        return bestOffer;
    }

    /**
     * Helper function for structuring and sending a CNP message. 
     * TODO needs to be changed in various ways. And maybe moved to communicationAid.
     * @param startTime
     * @param receivers
     * @return taskID
     */
    public int sendCNPmessage(double startTime, String startPos, ArrayList<Integer> receivers) {
        // taskID & agentID & pos & startTime 
        String startTimeStr = Double.toString(startTime);
        String body = this.robotID + this.separator + startPos + this.separator + startTimeStr;
        Message m = new Message(this.robotID, receivers, "cnp-service", body);
        return this.sendMessage(m, true);
    }

    protected Message handleOffers(int taskID){ return new Message(); }
}
