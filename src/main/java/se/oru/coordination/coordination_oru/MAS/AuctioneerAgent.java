package se.oru.coordination.coordination_oru.MAS;
import java.util.ArrayList;
import org.metacsp.multi.spatioTemporal.paths.Pose;

public class AuctioneerAgent extends BasicAgent{
    protected double TIME_WAITING_FOR_OFFERS = 3.0;


    protected Task generateTaskFromOffer(Message offerMsg, boolean setToActive){
        String[] msgParts = parseMessage(offerMsg, "", true);
        // replace intexes
        return new Task(Integer.parseInt(msgParts[0]), offerMsg.sender, setToActive, Double.parseDouble(msgParts[6]), Double.parseDouble(msgParts[4]),
                        Double.parseDouble(msgParts[5]), Double.parseDouble(msgParts[7]), this.posefyString(msgParts[2]), this.posefyString(msgParts[3]));
    }
    protected Task generateTaskFromOffer(Message offerMsg){
        return this.generateTaskFromOffer(offerMsg, true);
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

}
