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
    protected String separator = ".";
    protected int robotID;

    public ArrayList<Integer> robotsInNetwork = new ArrayList<Integer>();
    protected ArrayList<Message> offers = new ArrayList<Message>();

    public ArrayList<Message> inbox = new ArrayList<Message>();
    public ArrayList<Message> outbox = new ArrayList<Message>();

    public HashMap<Integer,String> activeTasks = new HashMap<Integer, String>();

    // messaging utils
    public Random rand = new Random(System.currentTimeMillis());

    //protected HashMap<Integer, ArrayList<Message>> auctions = new HashMap<Integer, ArrayList<Message>>();


    public void sendMessage(Message m){
        this.sendMessage(m, false); 
    }

    /**
     * Function for sending a message. 
     * If genTaskID is true. The function generates an id, converts it to a string and attaches it to the message. 
     * @param m the message to be sent
     * @param genTaskID if true we generate a taskID for the msg and log the task in this.activeTasks
     */
    public void sendMessage(Message m, boolean genTaskID){
        if (genTaskID){
            int taskID = this.tID();

            if (m.body == "") m.body = Integer.toString(taskID);
            else  m.body = taskID + this.separator + m.body;

            this.logTask(taskID, m.type);
        }
        
        synchronized(this.outbox){ this.outbox.add(m); }
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

    /** tID generates a new random task id.
     * returns an int between [this.lowerTaskIDBound , this.higherTaskIDBound]
     */
    public int tID(){ // generate a new task id bound 1000-9999
        return this.rand.nextInt((this.upperTaskIDBound - this.lowerTaskIDBound) + 1) + this.lowerTaskIDBound;
    }

    /**
     * Periodically checks the inbox of a robot.
     * Reacts to different messages depending on their message type.
     * -------------------------------------------------------------
     * if type == hello-world: add sending robot to its network
     * if type == res-offer: 
     * if type == accept: see taskHandler function. 
     */
    public void listener(){
        ArrayList<Message> inbox_copy;

        while(true){
        
            synchronized(inbox){
                inbox_copy = new ArrayList<Message>(this.inbox);
                this.inbox.clear();
            }

            for (Message m : inbox_copy){
                if (m.type == "hello-world"){
                    this.robotsInNetwork.add(m.sender);
                    this.sendMessage(
                        new Message( m.receiver.get(0), m.sender, "accept", m.body));
                } 

                else if (m.type == "res-offer"){
                    this.offers.add(m);
                }

                else if (m.type == "accept"){
                    this.taskHandler(Integer.parseInt(m.body), m);
                }
            }

            System.out.println(this.robotID + " -- " + this.robotsInNetwork);
            try { Thread.sleep(1000); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }

    }

    /**
     * activeTasks is a datastructure storing tasks with their id for a robot. 
     * This function checks if the taskID of a certain message is contained in the active tasks.
     * If: 
     * Task-type is "hello-world", the robot should add the accepting robot to its network. 
     * @param taskID
     * @param m
     */
    public void taskHandler(int taskID, Message m){
        String taskInfo = this.activeTasks.get(taskID);

        if (taskInfo == "hello-world" && !this.robotsInNetwork.contains(m.sender)){
            this.robotsInNetwork.add(m.sender);
        }

    }

    
    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link RobotAgent} calling this
     */
    public void offerService(int robotID){
        // skicka ut AD, vänta på erbjudanden i X sekunder

        this.offers.clear();

        ArrayList<Integer> receivers = new ArrayList<Integer>(this.robotsInNetwork);
        receivers.removeIf(i -> i<5000);    //storage agents has robotID > 5000

        // broadcast message to all storage agents
        Message m = new Message(robotID, receivers, "cnp-service-offer", "1,3,10");
        this.sendMessage(m);

        //sleep 6 sec before looking at offers
        try { Thread.sleep(6000); }
        catch (InterruptedException e) { e.printStackTrace(); }


        Message bestOffer = this.handleOffers(); //extract best offer
        
        // Send response: Mission to best offer sender, and deny all the other ones.
        Message acceptMessage = new Message(robotID, bestOffer.sender, "accept-offer", "");
        this.sendMessage(acceptMessage);

        receivers.remove(bestOffer.sender);

        Message declineMessage = new Message(robotID, receivers, "decline-offer", "");
        this.sendMessage(declineMessage);

        // vi har fått pose för storage som vill ha ore

        // gör mission: åker till ore extraction -> hämta ore -> åk till storage -> droppa ore.

        //lägg in i mitt schedule

        //klar

        // DO TASK !!!
        // gör new mission och lägg in i schedule 
    }


    
    /** handleService is called from within a storage agent, when a transport agent did a {@link offerService}
     * 
     * @param m the message with the service
     * @param robotID the robotID of this object
     * @return true if we send offer = we expect resp.
     */
    public boolean handleService(Message m, int robotID){ 
        boolean scheduleWorks = true;   // for test

        if (m.type != "cnp-service-offer") return false;

        // ########## For testing #################
        if ( scheduleWorks ) {
            Message resp = new Message(robotID, m.sender, "res-offer", "100");
            this.sendMessage(resp);
        }
        // räkna ut ett bud och skicka det.

        return true;
    }

    /**
     * HandleOffers is called from a transport agent, to either accept the offer of a storage agent, or deny it.
     * @return
     */
    public Message handleOffers(){

        Message bestOffer = new Message();
        int offerVal = 0;
        
        // Sort offers for the best one
        for ( Message m : this.offers ){
            int val = Integer.parseInt(m.body);
            if (val > offerVal){
                offerVal = val;
                bestOffer = m;
            }
        }

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
