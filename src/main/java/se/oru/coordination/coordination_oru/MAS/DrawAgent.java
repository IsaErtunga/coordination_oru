/**
 * 
 */
package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Arrays;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.MAS.Router;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.Mission;

public class DrawAgent extends CommunicationAid{
    // control parameters
    protected String COLOR = "\033[0;36m";
    protected double ADD_TIME2TA_TASKS = 0.0;

    private double finalXPos;
    protected double initalXPos;

    protected Pose pos;
    protected double amount;
    protected double capacity; 
    protected ReedsSheppCarPlanner mp;
    protected boolean shiftLeft;

    protected TimeScheduleNew timeSchedule;
    protected long startTime;

    public DrawAgent(int robotID, Router router, double capacity, Pose pos, ReedsSheppCarPlanner mp){}

    public DrawAgent(   int robotID, Router router, double capacity, Pose pos, ReedsSheppCarPlanner mp,
                        long startTime, boolean shiftLeft){

        this.robotID = robotID; // drawID >10'000
        this.capacity = capacity;
        this.amount = capacity; // 100% full in beginning
        this.mp = mp;
        this.pos = pos;
        this.initalXPos = pos.getX();

        this.timeSchedule = new TimeScheduleNew(pos, capacity, this.amount);
        this.startTime = startTime;

        router.enterNetwork(this.robotID, this.inbox, this.outbox);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);

        this.shiftLeft = shiftLeft;
        if (shiftLeft) {
            this.finalXPos = this.initalXPos - 32.0;
        } else {
            this.finalXPos = this.initalXPos + 32.0;
        }

    }

    protected double getTime(){
        long diff = System.currentTimeMillis() - this.startTime;
        return (double)(diff)/1000.0;
    }

    public void takeOre(double oreChange){
        oreChange = oreChange < 0.0 ? oreChange : -oreChange; // make sure sign is negative
        this.amount += oreChange;

        double x;
        if (this.shiftLeft) x = this.finalXPos + 32.0 * (this.amount / this.capacity);
        else x = this.finalXPos - 32.0 * (this.amount / this.capacity);

        this.pos = new Pose( x, pos.getY(), pos.getYaw() );
    }


    public void listener(){
        ArrayList<Message> inbox_copy;

        while(true){
        
            synchronized(this.inbox){
                inbox_copy = new ArrayList<Message>(this.inbox);
                this.inbox.clear();
            }

            for (Message m : inbox_copy){
                //System.out.println(m.type +"\t"+m.body);
                int taskID = Integer.parseInt(this.parseMessage(m, "taskID")[0]);
                
                if (m.type == "hello-world"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                    this.sendMessage( new Message( m.receiver.get(0), m.sender, "echo", Integer.toString(taskID)));
                }

                if (m.type == "echo"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                }

                else if (m.type == "accept") {

                    boolean eventAdded;
                    synchronized(this.timeSchedule){ eventAdded = this.timeSchedule.setEventActive(taskID); }
                    this.print("accept-msg, taskID-->"+taskID+"\twith robot-->"+m.sender+"\ttask added-->"+eventAdded);
                    if ( eventAdded == false ){
                        this.print("accept received but not successfully added. sending abort msg");
                        this.sendMessage(new Message(this.robotID, m.sender, "inform", taskID+this.separator+"abort"));
                    }
                } 

                else if (m.type == "decline"){
                    synchronized(this.timeSchedule){
                        boolean successfulRemove = this.timeSchedule.removeEvent(taskID);
                        //this.print("got decline from-->"+m.sender+"\ttaskID-->"+taskID+"\tremoved-->"+successfulRemove);
                    }
                }

                else if (m.type == "cnp-service"){
                    this.handleService(m);
                }

                else if (m.type.equals(new String("inform"))) {
                    this.handleInformMessage(m);
                }
                
            }
            // Changed sleep from 1000
            this.sleep(100);
        }
    }

    protected void handleInformMessage(Message m){
        int taskID = Integer.parseInt(this.parseMessage(m, "taskID")[0]);
        String informVal = this.parseMessage(m, "informVal")[0];
        
        if (informVal.equals(new String("done"))) {
            double oreChange = Double.parseDouble(this.parseMessage(m, "", true)[2]);
            this.timeSchedule.removeEvent(taskID);
            this.takeOre(oreChange);
        }

        else if (informVal.equals(new String("status"))) { //TODO change so schedule gets updated: newEndTime = Double.parseDouble(messageParts[2])
            // this.print("in status: ---SCHEDULE---");
            // this.timeSchedule.printSchedule(this.COLOR);

            double newEndTime = Double.parseDouble(this.parseMessage(m, "", true)[2]);
            Task taskToAbort = this.timeSchedule.updateTaskEndTimeIfPossible(taskID, newEndTime); // this function aborts task from schedule

            if ( taskToAbort != null ){
                this.sendMessage(new Message(this.robotID, taskToAbort.partner, "inform", taskToAbort.taskID+this.separator+"abort"));
                this.print("sending ABORT msg. taskID-->"+taskID+"\twith-->"+m.sender );
            }
            else {this.print("updated without conflict-->"+taskID +"\twith-->"+ m.sender);}
        }                     

        else if (informVal.equals(new String("abort"))) { //TODO remove task from schedule 
            this.timeSchedule.abortEvent(taskID);
            this.print("got ABORT MSG! taskID-->"+taskID+"\twith-->"+m.sender );
        } 
    }

    /**
     * DA responds to TA with offer that is calculated in this function. 
     * SCHEDULE:
     * - Will receive a time from TA of when it can come and fetch ore. 
     */
    public boolean handleService(Message m){ 
        double availabeOre = this.timeSchedule.getLastOreState();
        if (availabeOre <= 0.0) return false;   //if we dont have ore dont act 
        else availabeOre = availabeOre >= 15.0 ? 15.0 : availabeOre; // only give what ore we have available

        Task DAtask = createTaskFromServiceOffer(m, availabeOre);

        if ( !this.timeSchedule.isTaskPossible(DAtask) ) return false;    // task doesnt fit in schedule
        int offerVal = this.calculateOffer(DAtask, m);

        if ( offerVal <= 0 ) return false;
        if (! this.timeSchedule.addEvent(DAtask) ) return false;
        
        // this.print("--- schedule ---");
        // this.timeSchedule.printSchedule(this.COLOR);

        this.sendMessage(this.createOfferMsgFromTask(DAtask, offerVal, availabeOre));
        return true;
    }

    /**
     * Function that determines if DrawAgent moves right or left
     * And how much.
     * @param time
     * @return
     */
    protected Pose calculateFuturePos(double time){
        double oreAtTime = this.timeSchedule.getLastOreState(); //TODO this doesnt take ore into account. fix!
        double x;
        if (this.shiftLeft) x = this.finalXPos + 32.0 * (oreAtTime / this.capacity);
        else x = this.finalXPos - 32.0 * (oreAtTime / this.capacity);
        return new Pose( x, pos.getY(), pos.getYaw() );
    }

    /**
     * When receiving a cnp-message, this function will create a task from that message.
     * Only used in {@link handleService}.
     * @param m the message from the auctioneer
     * @param ore the amount of ore the task is about
     * @return a Task with attributes extracted from m
     */
    protected Task createTaskFromServiceOffer(Message m, double ore){
        String[] mParts = this.parseMessage(m, "", true);

        Pose TApos = this.posefyString(mParts[2]);

        // removed for easier debug
        // PoseSteering[] path = this.getPath( this.mp, this.pos, TApos);
        // double pathDist = this.calculatePathDist(path);
        // double pathTime = this.calculateDistTime(pathDist);

        // this.print("pathDist-->"+pathDist+"\twith-->"+m.sender);

        double pathDist = this.pos.distanceTo(TApos);
        double pathTime = this.calculateDistTime(pathDist) + 4.0;

        double taskStartTime = Double.parseDouble(mParts[3]);
        double endTime = taskStartTime + pathTime;

        Pose DApos = this.calculateFuturePos(taskStartTime);

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -ore, taskStartTime, endTime, pathTime, TApos, DApos);
    }

    /**
     * Used to generate a response message from a task. Called from {@link handleService}
     * after creating a task with {@link createTaskFromServiceOffer}.
     * @param t a Task that is unactive = t.isActive = false
     * @param offer an int that is the calculated evaluation of the service related to t
     * @param ore a double representing the ore amount the task handels
     * @return returns a Message with attributes extracted from the parameters
     */
    protected Message createOfferMsgFromTask(Task t, int offer, double ore){
        String s = this.separator;

        String TAposStr = this.stringifyPose(t.fromPose);
        String DAposStr = this.stringifyPose(t.toPose);
        String body = t.taskID +s+ offer +s+ TAposStr +s+ 
                      DAposStr +s+ t.startTime +s+ (t.endTime - this.ADD_TIME2TA_TASKS) +s+ ore;

        return new Message(this.robotID, t.partner, "offer", body);
    }

    /**
     * this function evaluates a task and calculates how good this task fits for this agent.
     * If this function determins that we dont want to participate in the auction this function
     * will return 0.
     * @param t the task to be evaluated how well it fits for this agent
     * @return an int with the value of this task.
     */
    protected int calculateOffer(Task t, Message m){
        if (t.pathDist <= 2.0) return 0;

        int fullOreBonus = t.ore < -15.0 +0.1 ? 1000 : 0;
        
        // distance calc
        double dist = t.fromPose.distanceTo(t.toPose);

        int distEval = this.calcCDF( dist, 500 );
        // time bonus
        double CNPstartTime = Double.parseDouble(this.parseMessage(m, "startTime")[0]);
        double timeDiff = Math.abs(t.startTime - CNPstartTime);
        int timeEval = (int)( 50 - timeDiff );

        //this.print("with robot-->"+m.sender +"\t dist-->"+dist+"\tdistance eval-->"+distEval+"\t cnp starttime-->"+CNPstartTime+"\t timeDiff-->"+timeDiff+"\ttime eval-->"+timeEval);
        return fullOreBonus + distEval + timeEval;
        
        // int offer;
        // if (t.pathDist > 0.5) {
        //     double evaluatedCapacity = 100.0 * this.amount / this.capacity; 
        //     if (this.amount / this.capacity > 0.9) {
        //         // Ändra med t.ore. If t.ore < -15
        //         // Draw agent is nearly full
        //         int fullOreBonus = 1000;
        //         offer = this.calcCDF(t.pathDist) + fullOreBonus;
        //     }
        //     else {
        //         offer = this.calcCDF(t.pathDist) + (int)evaluatedCapacity;
        //     }
        // }
        // else {
        //     offer = 0;
        // }
        // return offer;
    }

    /**
     * simlpe print func with color and robotID included
     * @param s string to be printed
     */
    protected void print(String s){
        System.out.println(this.COLOR+this.robotID+"\t" + s + "\033[0m");
    }
    
}
