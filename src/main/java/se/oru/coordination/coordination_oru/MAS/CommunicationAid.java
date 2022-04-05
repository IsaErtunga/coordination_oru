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

        }
        synchronized(this.outbox){ this.outbox.add(m); }
        return taskID;
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

            if (m.type == "accept" || m.type == "echo"){
                attributes = new String[] {"taskID"};
            }

            if (m.type == "offer"){
                attributes = new String[] {"taskID", "offerVal", "startPos", "endPos", "startTime", "endTime"};
            }

            else if (m.type == "inform") {
                attributes = new String[] {"taskID", "informVal", "oreChange"};
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
    public ArrayList<Integer> getReceivers(ArrayList<Integer> network, String receiverType) {
        ArrayList<Integer> networkCopy = new ArrayList<Integer>(network);
        if (receiverType.equals("DRAW")) {
            networkCopy.removeIf(i -> i < 10000);    //draw agents has robotID > 10000
        }
        if (receiverType.equals("STORAGE")) {
            networkCopy.removeIf(i -> i < 5000 && i >= 10000); //storage agents has robotID > 5000 & < 10000
        }
        if (receiverType.equals("TRANSPORT")) {
            networkCopy.removeIf(i -> i > 5000); //storage agents has robotID > 5000 & < 10000
        }
        return networkCopy;
    }

    /**
     * Call when need to sleep
     * @param ms
     */
    public void sleep(int ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { e.printStackTrace(); }
    }


    /**
     * Helper function that gets a pose and prepares it for message.
     * @param pose
     * @return
     */
    public String stringifyPose(Pose pose) {
        return pose.getX() + " " + pose.getY() + " " + pose.getYaw();
    }

    /**
     * Helper function to calculate endTime based on startTime and the path it took. 
     * @param startTime
     * @param path
     * @return
     */
    public double calculateEndTime(double startTime, PoseSteering[] path) {
          // Estimate path time
          double accumulatedDist = 0.0;
	    
          for (int i=0; i< path.length-1; i++) {
              Pose p1 = path[i].getPose();
              Pose p2 = path[i+1].getPose();
  
              double deltaS = p1.distanceTo(p2);
              accumulatedDist += deltaS;
          }
  
          double vel = 0.068;
          double estimatedPathTime = vel * accumulatedDist;
          double endTime = startTime + estimatedPathTime;
          return endTime;
    }


    /**
     * Helper function to calculate distance between to Pose objects. 
     * @param start
     * @param end
     * @return
     */
    protected double calcDistance(Pose start, Pose end) {
        return start.distanceTo(end);
    }

    /**
     * 
     * @param message
     * @return Task
     */
    protected Task createTaskFromMessage(Message message, double ore) {
        String[] msgParts = parseMessage(message, "", true);
        // replace intexes
        return new Task(msgParts[0], message.sender, null, false, ore, msgParts[4], msgParts[5], msgParts[2], msgParts[3]);
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
