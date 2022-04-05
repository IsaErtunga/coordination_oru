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
        System.out.println("===== Storage Constructor =====");
        this.robotID = r_id;
        this.capacity = capacity;
        this.amount = 0;
        this.startPose = startPos;
        this.timeSchedule = new TimeSchedule();
        this.startTime = startTime;

        router.enterNetwork(this);

        String type = "hello-world";
        this.sendMessage(new Message(this.robotID, type, ""), true);

    }


    protected double getTime(){
        System.out.println(this.robotID+"\ttime---> "+(System.currentTimeMillis() - this.startTime));
        long diff = System.currentTimeMillis() - this.startTime;
        return (double)(diff)/1000.0;
    }


    /**
     * 
     */
    public void status () {
        while(true) {
            if (this.timeSchedule.checkEndStateOreLvl() < this.oreStateThreshold && 
                this.timeSchedule.checkEndStateOreLvl() < capacity) {
                // SCHEDULE
                Message bestOffer = this.offerService();
                this.createTaskFromMessage(bestOffer);
            this.sleep(1000);
            }   
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
        
        if (this.amount + ore <= this.capacity && ore>0) this.amount = this.amount + ore;
    }

    public double dumpOre(double reqAmount){
        double ret = 0.0;
        
        if (reqAmount > 0){
            if (this.amount - reqAmount < 0){
                ret = this.amount;
                this.amount = 0.0;
            }
            else{
                ret = reqAmount;
                this.amount = this.amount - reqAmount;
            } 
        }
        return ret;
    }

    public void communicateState(){
        // coms

    }

    public void taskHandler(int taskID, Message m){
        String[] taskInfo = this.activeTasks.get(taskID).split(this.separator);

        if (taskInfo[0] == "hello-world" && !this.robotsInNetwork.contains(m.sender)){
            this.robotsInNetwork.add(m.sender);
        }

    }

    public void listener(){
        ArrayList<Message> inbox_copy;

        while(true){
        
            synchronized(inbox){
                inbox_copy = new ArrayList<Message>(this.inbox);
                this.inbox.clear();
            }

            for (Message m : inbox_copy){
                if (m.type == "hello-world"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                    this.sendMessage( new Message( m.receiver.get(0), m.sender, "echo", ""));
                }

                if (m.type == "echo"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                }

                else if (m.type == "offer"){
                    this.offers.add(m);
                }

                else if (m.type == "accept"){
                    this.taskHandler(Integer.parseInt(m.body), m);
                }

                
                else if (m.type == "inform") {
                    // TA informs SA when its done with a task.
                    String informVal = this.parseMessage(m, "informVal")[0]; 
                    String taskID = this.parseMessage(m, "taskID")[0];
                    Integer ore = Integer.parseInt(this.parseMessage(m, "oreChange")[0]);
    
                    if (informVal.equals(new String("abort"))) {
                        // Create a new task. 
                        // offerService();
                    } 
                    else if (informVal.equals(new String("done"))) {
                        System.out.println("****************** DONE ***********************");
                        if (ore >= 0) {
                            addOre(ore);
                        } else {
                            dumpOre(ore);
                        }
                    }
                    else if (informVal.equals(new String("result"))) {
                        
                    }
                    
                }
                
            }
            this.sleep(1000);
        }

    }

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportAgent} calling this
     */
    public Message offerService(){

        System.out.println(this.robotID +"======================1");

        ArrayList<Integer> receivers = this.getReceivers(this.robotsInNetwork, "TRANSPORT");
        
        System.out.println(this.robotID +"======================2");

        String startPos = this.stringifyPose(this.startPose);
 
        int taskID = this.createCNPMessage(this.timeSchedule.getNextStartTime(), startPos, receivers);
        System.out.println(this.robotID +"======================3");

        //sleep  before looking at offers
        this.sleep(2500);

        System.out.println(this.robotID +"======================4");

        Message bestOffer = this.handleOffers(taskID); //extract best offer
        System.out.println(this.robotID +"======================5");

        if (!bestOffer.isNull){        
            // Send response: Mission to best offer sender, and deny all the other ones.
            Message acceptMessage = new Message(robotID, bestOffer.sender, "accept", Integer.toString(taskID) );
            this.sendMessage(acceptMessage);

            receivers.removeIf(i -> i==bestOffer.sender);    //storage agents has robotID > 5000
    
            // Send decline message to all the others. 
            Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(taskID));
            this.sendMessage(declineMessage);

            //TODO add amout A to be received at time T in schedule

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
    public int createCNPMessage(double startTime, String startPos, ArrayList<Integer> receivers) {
        // taskID & agentID & pos & startTime 
        String startTimeStr = Double.toString(startTime);
        String body = this.robotID + this.separator + startPos + this.separator + startTime;
        Message m = new Message(this.robotID, receivers, "cnp-service", body);
        return this.sendMessage(m, true);
    }

       /**
     * HandleOffers is called from a SA, to either accept the offer of a TA, or deny it.
     * @return the message that is the best offer
     */
    public Message handleOffers(int taskID) {

        Message bestOffer = new Message();
        int offerVal = 0;
        
        // Sort offers for the best one

        for (Message m : this.offers) {
            // SCHEDULE: Extract startTime & endTime and see if it fits into schedule
            double startTime = Double.parseDouble(parseMessage(m, "startTime")[0]);
            double endTime = Double.parseDouble(parseMessage(m, "endTime")[0]);
  
            if(!m.isNull && this.timeSchedule.taskPossible(startTime, endTime)) {
                String[] mParts = this.parseMessage( m, "", true); // sort out offer not part of current auction(taskID)
                if (Integer.parseInt(mParts[0]) == taskID){
                    int val = Integer.parseInt(mParts[1]);
                    if (val > offerVal){
                        offerVal = val;
                        bestOffer = new Message(m);
                    }
                    //this.offers.remove(m);
                }
            }
        }
        //TODO make it able to choose another offer if OG one was not possible
        return bestOffer;
    }

}
