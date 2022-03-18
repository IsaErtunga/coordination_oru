/**
 * @author Alexander Benteby & Isa Ertunga
 */

package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import se.oru.coordination.coordination_oru.MAS.Message;


public class CommunicationAid {

    protected int lowerTaskIDBound = 1000;
    protected int upperTaskIDBound = 9999;
    protected String separator = ",";
    public int robotID;

    public ArrayList<Integer> robotsInNetwork = new ArrayList<Integer>();
    protected ArrayList<Message> offers = new ArrayList<Message>();

    public ArrayList<Message> inbox = new ArrayList<Message>();
    public ArrayList<Message> outbox = new ArrayList<Message>();

    public HashMap<Integer,String> activeTasks = new HashMap<Integer, String>();

    // messaging utils
    public Random rand = new Random(System.currentTimeMillis());
    


    // robot?
    public void listener2(){
        //this.taskHandler()
        //this.parseMessage()
        //this.offerService()
        //this.handleService()
    }
    public void taskHandler2(int taskID, Message m){} // in robot?
    
    // comm aid
    public void sendMsg2(Message m){}
    public void logTask2(int taskID, String info){}
    public String[] parseMessage2(Message m, String get, boolean retriveAll){ return null;}
    public int tID2(){ return 0;}

    // CNP
    public void offerService2(){}
    public boolean handleService2(Message m){ return true; }
    public Message handleOffers2(int taskID){return null;}




    public int sendMessage(Message m){
        return this.sendMessage(m, false); 
    }
    /**
     * Function for sending a message. 
     * If genTaskID is true. The function generates an id, converts it to a string and attaches it to the message. 
     * @param m the message to be sent
     * @param genTaskID if true we generate a taskID for the msg and log the task in this.activeTasks
     */
    public int sendMessage(Message m, boolean genTaskID){
        int taskID = -1;

        if (genTaskID){
            taskID = this.tID();

            if (m.body == "") m.body = Integer.toString(taskID);
            else  m.body = taskID + this.separator + m.body;

            this.logTask(taskID, m.type);
        }
        synchronized(this.outbox){ this.outbox.add(m); }
        return taskID;
    }
    
    /** logTask will add a task to this.activeTasks to keep context for conversations
     * to remember what we sent and what responses we expect.
     * 
     * @param taskID the task id to be added to this.activeTasks
     * @param info  information about the taskID to give context
     */
    public void logTask(int taskID, String info){
        synchronized(this.activeTasks){ this.activeTasks.put(taskID, info); }
    }

    public String[] parseMessage(Message m, String get){
        return this.parseMessage(m, get, false);
    }

    public String[] parseMessage(Message m, String get, boolean retriveAll){
        String[] parts = m.body.split(this.separator);

        if (retriveAll){
            return parts;
        }
        else {
            String[] attributes = {};

            if (m.type == "offer"){
                attributes = new String[] {"taskID", "offerVal"};
            }

            else if (m.type == "inform") {
                attributes = new String[] {"taskID", "informVal"};
            }

            else if (m.type == "cnp-service"){
                attributes = new String[] {"taskID", "storageID", "pos"}; //TODO add all attributes
            }

            int i = 0;
            for (String attribute : attributes){
                if (get == attribute) return new String[]{parts[i]};
                i = i+1;
            }
    }

        return new String[] {}; // default
    }

    /** tID generates a new random task id.
     * returns an int between [this.lowerTaskIDBound , this.higherTaskIDBound]
     */
    public int tID(){ // generate a new task id bound 1000-9999
        return this.rand.nextInt((this.upperTaskIDBound - this.lowerTaskIDBound) + 1) + this.lowerTaskIDBound;
    }

    

    /** #######################################################################
     *  ######################## Contract Net Protocol ########################
        ####################################################################### */ 
    
    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link RobotAgent} calling this
     */
    public Message offerService(){

        System.out.println("======================1");

        ArrayList<Integer> receivers = new ArrayList<Integer>(this.robotsInNetwork);
        receivers.removeIf(i -> i>5000);    //storage agents has robotID > 5000
        System.out.println("======================2");

        // broadcast message to all transport agents
        //Pose pos = new Pose(63.0,68.0, 0.0);
        String body = this.robotID + this.separator + "63.0 68.0 0.0";
        Message m = new Message(this.robotID, receivers, "cnp-service", body);
        int taskID = this.sendMessage(m, true);
        System.out.println("======================3");

        //sleep 6 sec before looking at offers
        try { Thread.sleep(2500); }
        catch (InterruptedException e) { e.printStackTrace(); }
        System.out.println("======================4");

        Message bestOffer = this.handleOffers(taskID); //extract best offer
        System.out.println("======================5");

        if (bestOffer != null){        
            // Send response: Mission to best offer sender, and deny all the other ones.
            Message acceptMessage = new Message(robotID, bestOffer.sender, "accept", Integer.toString(taskID) );
            this.sendMessage(acceptMessage);

            receivers.removeIf(i -> i==bestOffer.sender);    //storage agents has robotID > 5000
    
            // Send decline message to all the others. 
            Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(taskID));
            this.sendMessage(declineMessage);

            //TODO add amout A to be received at time T in schedule

            return bestOffer;
        }
        return null;
    }

    
    /** handleService is called from within a TA, when a TA did a {@link offerService}
     * 
     * @param m the message with the service
     * @param robotID the robotID of this object
     * @return true if we send offer = we expect resp.
     */
    public boolean handleService(Message m){ 

        if (m.type != "cnp-service") return false;

        String[] mParts = this.parseMessage( m, "", true);
        int offer = this.tID();         //TODO current is for test
        String body = mParts[0] + this.separator + offer;

        Message resp = new Message(this.robotID, m.sender, "offer", body);
    
        // räkna ut ett bud och skicka det.
        this.sendMessage(resp);
        this.logTask(Integer.parseInt(mParts[0]),
            "offer" + this.separator + m.sender + this.separator + mParts[2] ); //TODO make better
        
        System.out.println(this.robotID + ", task: " + this.activeTasks.get(Integer.parseInt(mParts[0])));
        
        return true;
    }

    /**
     * HandleOffers is called from a SA, to either accept the offer of a TA, or deny it.
     * @return the message that is the best offer
     */
    public Message handleOffers(int taskID){

        Message bestOffer = new Message();
        int offerVal = 0;
        
        // Sort offers for the best one
        for ( Message m : this.offers ){

            String[] mParts = this.parseMessage( m, "", true); // sort out offer not part of current auction(taskID)
            if (Integer.parseInt(mParts[0]) != taskID) continue;
                
            int val = Integer.parseInt(mParts[1]);
            if (val > offerVal){
                offerVal = val;
                bestOffer = m;
            }
        }
        System.out.println("in handleOffers: " + bestOffer.type +":"+bestOffer.body);

        //TODO make it able to choose another offer if OG one was not possible
        return bestOffer;
    }
    
    public static void main(String[] args){
        /*
        RobotAgent r1 = new RobotAgent(0);
        RobotAgent r2 = new RobotAgent(1);

        Router router = new Router();

        router.enterNetwork(r1);
        router.enterNetwork(r2);

        r1.outbox.add( new Message(r1.robotID, "hello-world", Integer.toString(r1.robotID)) );
        r2.outbox.add( new Message(r2.robotID, "hello-world", Integer.toString(r2.robotID)) );

        Thread t = new Thread() {
			public void run() {
				r1.listener();
            }
        };
        t.start();

        Thread t2 = new Thread() {
			public void run() {
				r2.listener();
            }
        };
        t2.start();

        Thread t3 = new Thread() {
			public void run() {
				router.run();
            }
        };
        t3.start();

           */ 
    }



}
