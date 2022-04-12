package se.oru.coordination.coordination_oru.MAS;
import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.MAS.Router;


public class StorageAgent extends CommunicationAid{
    //Control parameters
    protected double TIME_WAITING_FOR_OFFERS = 3.0;
    protected String COLOR = "\033[1;33m";



    protected Pose startPose;
    protected double capacity;  // capacity of storage = max ore it can store in TONS
    protected double amount;    // the current amount it has stored in TONS

    protected TimeSchedule timeSchedule;
    protected long startTime;
    protected double oreStateThreshold = 15.0;

    public boolean beingUsed = false;

    // for testing
    public StorageAgent(int id){this.robotID = id;}     
    public StorageAgent(int r_id, Router router, double capacity, Pose startPos){}

    /**
     * Constructor for Storage Agent
     * @param r_id
     * @param router
     * @param capacity
     * @param startPos
     */
    public StorageAgent(int r_id, Router router, double capacity, Pose startPos, long startTime){
        this.print("STORAGE AGENT INITIATED");
        this.robotID = r_id;
        this.capacity = capacity;
        this.amount = capacity / 2.0;
        this.startPose = startPos;
        this.timeSchedule = new TimeSchedule(startPos, this.amount);
        this.startTime = startTime;

        router.enterNetwork(this);

        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);
    }


    protected double getTime(){
        long diff = System.currentTimeMillis() - this.startTime;
        return (double)(diff)/1000.0;
    }

    protected double getNextTime(){
        double STARTUP_ADD = 5.0;
        double nextTime = this.timeSchedule.getNextStartTime();
        return nextTime == -1.0 ? this.getTime()+STARTUP_ADD : nextTime;
    }

    public void status () {
        while(true) {
            if ( this.amount < this.oreStateThreshold && false ){} //TODO request ore fast


            else if ( false ){} //TODO check if we need more or at some point in future


            else if ( this.timeSchedule.checkEndStateOreLvl() < 0.9*capacity ){ // plan future tasks

                Message bestOffer = this.offerService(this.getNextTime());

                if (!bestOffer.isNull) {
                    Task task = this.createTaskFromMessage(bestOffer, true);
                    this.timeSchedule.add(task);
                    this.timeSchedule.printSchedule();
                }
            }
            this.sleep(2000);
        }      
    }

    public void start(){
        StorageAgent This = this;
        Thread listener = new Thread() {
            public void run() {
                This.listener();
            }
        };
        listener.start();

        this.sleep(5000);

        this.status();

    }

    public void addOre(double ore){
        ore = ore > 0.0 ? ore : -ore; // give ore + sign
        if (this.amount + ore <= this.capacity && ore>0) this.amount = this.amount + ore;
    }

    public double dumpOre(double reqAmount){
        reqAmount = reqAmount > 0.0 ? reqAmount : -reqAmount; // give reqAmount + sign
        
        if (this.amount - reqAmount < 0){
            double ret = this.amount;
            this.amount = 0.0;
            return ret;
        }

        this.amount = this.amount - reqAmount;
        return reqAmount;
    }

    public void listener(){
        ArrayList<Message> inbox_copy;

        while(true){
            synchronized(inbox){
                inbox_copy = new ArrayList<Message>(this.inbox);
                this.inbox.clear();
            }

            for (Message m : inbox_copy){
                int taskID = Integer.parseInt(this.parseMessage(m, "taskID")[0]);
                
                if (m.type == "hello-world"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                    this.sendMessage( new Message( m.receiver.get(0), m.sender, "echo", Integer.toString(taskID)));
                }

                else if (m.type == "echo"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                }

                else if (m.type == "offer"){
                    this.offers.add(m);
                }

                else if (m.type == "accept"){} //TODO does nothing in our Scenario atm

                
                else if (m.type == "inform") {
                    this.handleInformMessage(m);
                }
                
            }
            this.sleep(1000);
        }

    }

    /** this function holds the logic for handeling messages with type 'inform'. 
     * 
     * @param m the message with the 'inform'-type.
     */
    protected void handleInformMessage(Message m){
        int taskID = Integer.parseInt(this.parseMessage(m, "taskID")[0]);
        String informVal = this.parseMessage(m, "informVal")[0];
        
        if (informVal.equals(new String("done"))) {
            double oreChange = Double.parseDouble(this.parseMessage(m, "", true)[2]);
            this.timeSchedule.remove(taskID);

            if (oreChange > 0) this.addOre(oreChange);
                
            else this.dumpOre(oreChange);
        }

        else if (informVal.equals(new String("status"))) {} //TODO change so schedule gets updated: newEndTime = Double.parseDouble(messageParts[2])                    

        else if (informVal.equals(new String("abort"))) {} //TODO remove task from schedule 
    }

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportAgent} calling this
     */
    public Message offerService(double startTime){
        ArrayList<Integer> receivers = this.getReceivers(this.robotsInNetwork, "TRANSPORT");

        if ( receivers.size() <= 0 ) return new Message();
        
        this.offers.clear();
        int taskID = this.sendCNPmessage(startTime, this.stringifyPose(this.startPose), receivers);

        double time = this.waitForAllOffersToCome(receivers.size());  //locking function. wait for receivers
        this.print("time waited for offers-->"+time);

        Message bestOffer = this.handleOffers(taskID); //extract best offer

        if (!bestOffer.isNull){        
            // Send response: Mission to best offer sender, and deny all the other ones.
            Message acceptMessage = new Message(robotID, bestOffer.sender, "accept", Integer.toString(taskID) );
            this.sendMessage(acceptMessage);

            receivers.removeIf(i -> i==bestOffer.sender);

            if (receivers.size() > 0){
                // Send decline message to all the others. 
                Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(taskID));
                this.sendMessage(declineMessage);

            }
        }
        return bestOffer;
    }

    /** This function is locking. will wait for offers from all recipients to arrive OR return on the time delay.
     * 
     * @param nrOfReceivers an int with the number of recipients of the cnp message sent. 
     * @return a double with the time it took before exiting this function.
     */
    protected double waitForAllOffersToCome(int nrOfReceivers){
        double before = this.getTime();

        while ( this.offers.size() < nrOfReceivers && this.getTime() - before <= this.TIME_WAITING_FOR_OFFERS){
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
            double startTime = Double.parseDouble(parseMessage(m, "startTime")[0]);
            double endTime = Double.parseDouble(parseMessage(m, "endTime")[0]);
  
            if( this.timeSchedule.taskPossible(startTime, endTime) ) {
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

    /**simlpe print func with color and robotID included
     * @param s string to be printed
     */
    protected void print(String s){
        System.out.println(this.COLOR+this.robotID+"\t" + s + "\033[0m");
    }

}
