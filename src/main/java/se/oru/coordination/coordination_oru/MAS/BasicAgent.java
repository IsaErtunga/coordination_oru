package se.oru.coordination.coordination_oru.MAS;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import java.util.ArrayList;
import java.util.Random;

public class BasicAgent extends HelpFunctions{
    // control params
    protected int taskCap;

    protected int robotID;
    protected boolean goOffline = false;
    protected ReedsSheppCarPlanner mp;
    protected Pose initialPose;
    protected double capacity; // how much ore this agent can carry.
    protected double initialOreAmount; // how much ore this agent can carry.
    protected String COLOR = "";

    // for communication
    protected Router router;
    public ArrayList<Message> inbox = new ArrayList<Message>();
    public ArrayList<Message> outbox = new ArrayList<Message>();
    protected ArrayList<Integer> robotsInNetwork = new ArrayList<Integer>();
    protected String separator = ",";
    protected int lowerTaskIDBound = 1000;
    protected int upperTaskIDBound = 99999;

    // for time
    public Random rand = new Random(System.currentTimeMillis());
    protected long clockStartTime;
    protected TimeScheduleNew timeSchedule;

    // for auction
    protected ArrayList<Message> offers = new ArrayList<Message>();

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

    public int sendMessage(Message m){
        return this.sendMessage(m, false); 
    }

    public String[] parseMessage(Message m, String get, boolean retriveAll){
        String[] parts = m.body.split(this.separator);
        if (retriveAll) return parts;

        String[] attributes = {};

        if ( m.type.equals(new String("accept")) ||
                m.type.equals(new String("echo")) ||
                m.type.equals(new String("hello-world")) ||
                m.type.equals(new String("goodbye-world")) ||
                m.type.equals(new String("decline")) ||
                m.type.equals(new String("accept")) ){
            
            // do something
            attributes = new String[] {"taskID"};

        } else if ( m.type.equals(new String("offer")) ){
            attributes = new String[] {"taskID", "offerVal", "startPos", "endPos", "startTime", "endTime", "ore"};

        } else if (  m.type.equals(new String("inform")) ){
            attributes = new String[] {"taskID", "informVal", "informInfo"};

        } else if ( m.type.equals(new String("cnp-service")) ){
            attributes = new String[] {"taskID", "storageID", "startPos", "startTime"};
        }

        int i = 0;
        for (String attribute : attributes){
            if (get == attribute) return new String[]{parts[i]};
            i = i+1;
        }

        return new String[] {}; // default
    }
    public String[] parseMessage(Message m, String get){
        return this.parseMessage(m, get, false);
    }
    public String[] parseMessage(Message m){
        return this.parseMessage(m, "", true);
    }

    public int getMessageTaskID(Message m){
        return Integer.parseInt(this.parseMessage(m, "taskID")[0]);
    }

    /**
     * tID generates a new random task id.
     * returns an int between [this.lowerTaskIDBound , this.higherTaskIDBound]
     */
    public int tID() { // generate a new task id bound 1000-9999
        return this.rand.nextInt((this.upperTaskIDBound - this.lowerTaskIDBound) + 1) + this.lowerTaskIDBound;
    }

    /**
     * simlpe print func with color and robotID included
     * @param s string to be printed
     */
    protected void print(String s){
        System.out.println(this.COLOR+this.robotID+" TIME["+String.format("%.2f",this.getTime()) + "]\t" + s + "\033[0m");
    }

    protected double getTime(){
        long diff = System.currentTimeMillis() - this.clockStartTime;
        return (double)(diff)/1000.0;
    }

    /**
     * Helper function to get the right receivers.
     * Called in the initial communication phase. 
     * @param network 
     * @param receiverType TRANSPORT, STORAGE, DRAW
     * @return
     */
    public ArrayList<Integer> getReceivers(String receiverType) {
        /* ex: 1110:  first 1 = block 1, second 1 = transportAgent, third 1 = unique id

        agentType: 1=DA, 2=TA, 3=SA, 4TTA

        2310, block2, SA, id=10
        */
        int block = this.robotID / 1000;
        ArrayList<Integer> networkCopy = new ArrayList<Integer>(this.robotsInNetwork);
        networkCopy.removeIf(i -> i/1000 != block);

        if (receiverType.equals("DRAW")) {
            networkCopy.removeIf(i -> (i%1000)/100 != 1);

        } else if (receiverType.equals("STORAGE")) {
            networkCopy.removeIf(i -> (i%1000)/100 != 3);

        } else if (receiverType.equals("TRANSPORT")) {
            networkCopy.removeIf(i -> (i%1000)/100 != 2);

        } else if (receiverType.equals("TRANSPORTTRUCK")) {
            networkCopy.removeIf(i -> (i%1000)/100 != 4);
        }
        
        return networkCopy;
    }

    public void listener(){
        ArrayList<Message> inbox_copy;
        while(true){
            if ( this.goOffline ){
                this.sendMessage(new Message(this.robotID, "goodbye-world", ""), true );
                this.sleep(5000);
                this.router.leaveNetwork(this.robotID);
            }
            synchronized(this.inbox){
                inbox_copy = new ArrayList<Message>(this.inbox);
                this.inbox.clear();
            }

            for (Message m : inbox_copy){
                int taskID = this.getMessageTaskID(m);

                if ( m.type.equals(new String("hello-world")) ){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                    this.sendMessage( new Message( m.receiver.get(0), m.sender, "echo", Integer.toString(taskID)));

                } else if ( m.type.equals(new String("goodbye-world")) ){
                    for (int i=0; i<this.robotsInNetwork.size(); i++){
                        if ( this.robotsInNetwork.get(i) == m.sender ) this.robotsInNetwork.remove(i);
                    } 

                } else if ( m.type.equals(new String("echo")) ){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);

                } else if ( m.type.equals(new String("accept")) ){
                   this.handleAccept(taskID, m);

                } else if ( m.type.equals(new String("decline")) ){
                    synchronized(this.timeSchedule){
                        boolean successfulRemove = this.timeSchedule.removeEvent(taskID);
                        this.print("got decline from-->"+m.sender+"\ttaskID-->"+taskID+"\tremoved-->"+successfulRemove);
                    }

                } else if ( m.type.equals(new String("cnp-service")) ){
                    this.handleCNPauction(m);

                } else if (m.type.equals(new String("inform")) ){
                    this.handleInform(taskID, m);

                } else if (m.type.equals(new String("offer")) ){
                this.offers.add(m);
                }
            }
            // Changed sleep from 1000
            this.sleep(100);
        }
    }

    protected void handleInform(int taskID, Message m){
        String informVal = this.parseMessage(m, "informVal")[0];
        
        if (informVal.equals(new String("done"))) {
            this.handleInformDone(taskID, m);

        } else if (informVal.equals(new String("status"))) {
            this.handleInformStatus(m);

        } else if (informVal.equals(new String("abort"))) {
            this.timeSchedule.abortEvent(taskID);
            this.print("got ABORT MSG! taskID-->"+taskID+"\twith-->"+m.sender );
        }
    }


    protected void handleAccept(int taskID, Message m){};
    protected void handleInformDone(int taskID, Message m){};
    protected void handleInformStatus(Message m){};
    protected void handleCNPauction(Message m){};

}
