package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import se.oru.coordination.coordination_oru.MAS.Message;


public class CommunicationAid {

    public ArrayList<Integer> robotsInNetwork = new ArrayList<Integer>();
    protected ArrayList<Message> offers = new ArrayList<Message>();

    public ArrayList<Message> inbox = new ArrayList<Message>();
    public ArrayList<Message> outbox = new ArrayList<Message>();

    // messaging utils
    public Random rand = new Random(System.currentTimeMillis());

    //protected HashMap<Integer, ArrayList<Message>> auctions = new HashMap<Integer, ArrayList<Message>>();


    public void sendMessage(Message m){
        synchronized(this.outbox){ this.outbox.add(m); }
    }

    public int tID(){ // generate a new task id bound 1000-9999
        return this.rand.nextInt((9999 - 1000) + 1) + 1000;
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
                    this.robotsInNetwork.add(m.sender);
                    this.sendMessage(
                        new Message( m.receiver.get(0), m.sender, "accept", m.body));
                } 

                else if (m.type == "res-offer"){
                    this.offers.add(m);
                }
            }

            //System.out.println(this.robotsInNetwork);
            try { Thread.sleep(1000); }
            catch (InterruptedException e) { e.printStackTrace(); }
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

        
    }


}
