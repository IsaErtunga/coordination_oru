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
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;


public class CommunicationAid extends HelpFunctions{
    protected int lowerTaskIDBound = 1000;
    protected int upperTaskIDBound = 99999;
    protected String separator = ",";
    public int robotID;
    protected long startTime;

    public ArrayList<Integer> robotsInNetwork = new ArrayList<Integer>();
    protected ArrayList<Message> offers = new ArrayList<Message>();

    public ArrayList<Message> inbox = new ArrayList<Message>();
    public ArrayList<Message> outbox = new ArrayList<Message>();

    // messaging utils
    public Random rand = new Random(System.currentTimeMillis());
    

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

        }
        synchronized(this.outbox){ this.outbox.add(m); }
        this.fp.addMessageCounter(this.getTime(), m.type);
        return taskID;
    }

    public int sendMessage(Message m){
        return this.sendMessage(m, false); 
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

            if (m.type == "accept" || m.type == "decline" || m.type == "echo" || m.type == "hello-world"){
                attributes = new String[] {"taskID"};
            }

            if (m.type == "offer"){
                attributes = new String[] {"taskID", "offerVal", "startPos", "endPos", "startTime", "endTime", "ore"};
            }

            else if (m.type == "inform") {
                attributes = new String[] {"taskID", "informVal", "informInfo"};
            }

            else if (m.type == "cnp-service"){
                attributes = new String[] {"taskID", "storageID", "startPos", "startTime"}; //TODO add all attributes
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
    public int tID() { // generate a new task id bound 1000-9999
        return this.rand.nextInt((this.upperTaskIDBound - this.lowerTaskIDBound) + 1) + this.lowerTaskIDBound;
    }

    /**
     * Helper function to get the right receivers.
     * Called in the initial communication phase. 
     * @param network 
     * @param receiverType TRANSPORT, STORAGE, DRAW
     * @return
     */
    public ArrayList<Integer> getReceivers(int robotID, ArrayList<Integer> network, String receiverType) {
        /* ex: 1110:  first 1 = block 1, second 1 = transportAgent, third 1 = unique id

        agentType: 1=DA, 2=TA, 3=SA, 4TTA

        2310, block2, SA, id=10
        */
        int block = robotID / 1000;

        ArrayList<Integer> networkCopy = new ArrayList<Integer>(network);

        networkCopy.removeIf(i -> i/1000 != block);


        if (receiverType.equals("DRAW")) {
            networkCopy.removeIf(i -> (i%1000)/100 != 1);

        }
        else if (receiverType.equals("STORAGE")) {
            networkCopy.removeIf(i -> (i%1000)/100 != 3);
        }
        else if (receiverType.equals("TRANSPORT")) {
            networkCopy.removeIf(i -> (i%1000)/100 != 2);
        }
        else if (receiverType.equals("TRANSPORTTRUCK")) {
            networkCopy.removeIf(i -> (i%1000)/100 != 4);
        }
        return networkCopy;
    }

    /**
     * body -> int taskID & int offerVal & startPos & endPos & startTime & endTime & ore

     * @param message
     * @return Task
     */
    protected Task createTaskFromMessage(Message message) {
        String[] msgParts = parseMessage(message, "", true);
        // replace intexes
        return new Task(Integer.parseInt(msgParts[0]), message.sender, true, Double.parseDouble(msgParts[6]), Double.parseDouble(msgParts[4]),
                        Double.parseDouble(msgParts[5]), this.posefyString(msgParts[2]), this.posefyString(msgParts[3]));
    }
    

    protected double getTime(){
        long diff = System.currentTimeMillis() - this.startTime;
        return (double)(diff)/1000.0;
    }
    

    public static void main(String[] args){
        int robotID = 3210;
        int block = robotID / 1000;
        //int rest = robotID % 100;
        int agentType = (robotID % 1000) / 100;

        System.out.println("id-->"+robotID+"\tblock-->"+block+"\tagentType-->"+agentType );
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
