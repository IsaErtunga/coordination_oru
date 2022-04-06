/**
 * 
 */
package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.Arrays;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.MAS.Router;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.Mission;

public class DrawAgent extends CommunicationAid{

    private final static double finalXPos = 4.0;
    protected double initalXPos;

    protected Pose pos;
    protected double amount;
    protected double capacity; 
    protected ReedsSheppCarPlanner mp;

    protected TimeSchedule timeSchedule;
    protected long startTime;

    public DrawAgent(int robotID, Router router, double capacity, Pose pos, ReedsSheppCarPlanner mp){}

    public DrawAgent(int robotID, Router router, double capacity, Pose pos, ReedsSheppCarPlanner mp, long startTime){
        this.robotID = robotID; // drawID >10'000
        this.capacity = capacity;
        this.amount = capacity; // 100% full in beginning
        this.mp = mp;
        this.pos = pos;
        this.initalXPos = pos.getX();

        this.timeSchedule = new TimeSchedule();
        this.startTime = startTime;

        router.enterNetwork(this.robotID, this.inbox, this.outbox);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);

    }


    protected double getTime(){
        System.out.println(this.robotID+"\ttime---> "+(System.currentTimeMillis() - this.startTime));
        long diff = System.currentTimeMillis() - this.startTime;
        return (double)(diff)/1000.0;
    }

    public void takeOre(double oreChange){
        // alter ore amount
        if (this.amount + oreChange > 0.0) this.amount += oreChange;
        else this.amount = 0; //TODO fix case, TA can think it gets 15 tons but dont

        double x = this.finalXPos + (this.initalXPos - this.finalXPos) * this.amount / this.capacity;
        this.pos = new Pose( x, pos.getY(), pos.getYaw() );

        System.out.println(this.robotID + " oreChange, new pos -------------->" + this.pos.toString());

    }

    public void taskHandler(int taskID, Message m){
        String[] taskInfo = this.activeTasks.get(taskID).split(this.separator);

        if (taskInfo[0] == "hello-world" && !this.robotsInNetwork.contains(m.sender)){
            this.robotsInNetwork.add(m.sender);
        }

        else if(taskInfo[0].equals("offer")){   // we sent an offer to a SA and got accept reply
            System.out.println(this.robotID + ", in taskhandler: " + taskInfo);
            System.out.println(Arrays.toString(taskInfo));
            //TODO does this agent do something? I think it does nothing.

            // log pick up in schedule.
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
                System.out.println(m.type +"\t"+m.body+"\tparseRes: "+ this.parseMessage(m, "taskID"));
                int taskID = Integer.parseInt(this.parseMessage(m, "taskID")[0]);
                
                if (m.type == "hello-world"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                    this.sendMessage( new Message( m.receiver.get(0), m.sender, "echo", Integer.toString(taskID)));
                }

                if (m.type == "echo"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                }

                else if (m.type == "accept"){
                    // SCHEDULE: Change isActive.
                    this.timeSchedule.setTaskActive(taskID);
                }

                else if (m.type == "decline"){
                    //TODO add case
                }

                else if (m.type == "cnp-service"){
                    this.handleService(m);
                }

                else if (m.type.equals(new String("inform"))) {
                    // TA informs SA when its done with a task.
                    String[] messageParts = this.parseMessage(m, "", true);
                    String informVal = messageParts[1];
                    
                    if (informVal.equals(new String("done"))) {
                        /* SCHEDULE: 
                            * If early just remove from schedule.
                        */ 
                        Double oreChange = Double.parseDouble(messageParts[2]); 
                        this.timeSchedule.remove(taskID);
                        this.takeOre(oreChange);
                    }

                    else if (informVal.equals(new String("status"))) {
                        /* SCHEDULE: 
                            * if TA notice it will not be done in time, we get inform->status msg
                            * update schedule with new time and check if problem
                            * if 2 mission have big overlap then send ABORT msg to later mission.
                            * else all is good.
                        */ 
                        double newEndTime = Double.parseDouble(messageParts[2]);
                        if (newEndTime > this.timeSchedule.get(taskID).endTime) {
                            this.timeSchedule.update(taskID, newEndTime);
                        }
                    }

                    else if (informVal.equals(new String("abort"))) {
                        /* SCHEDULE:
                            * remove task from schedule
                        */
                    } 

                }
                
            }

            //System.out.println(this.robotID + " -- " + this.robotsInNetwork);
            try { Thread.sleep(1000); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
    }

    /**
     * DA responds to TA with offer that is calculated in this function. 
     * SCHEDULE:
     * - Will receive a time from TA of when it can come and fetch ore. 
     */
    public boolean handleService(Message m){ 
        if (m.type != "cnp-service") return false;

        // SCHEDULE: Need time in the message from TA, need to extend with ore. 
        String[] mParts = this.parseMessage(m, "", true); //parse=[ taskID, agentID, pos, startTime ]
        double ore = 15.0;

        // get pose of TA

        Pose TApos = this.posefyString(mParts[2]);

        //calc euclidean dist between DA -> TA, and capacity evaluation
        
        //TODO also include schedule: look if other agent will collect ore here at same time.
        //TODO add poseSteering.length

        // SCHEDULE: Need to lookup schedule too see if task is possible. 
        /* SCHEDULE: offer calc will include 
            * calc path from TA to DA
            * estimate the time TA will be using this tunnel
            * look in schedule if there is a time window for the task to fit in. 
            * - IF true: send offer & reserve time (Insert into schedule list)
            * - else: dont send offer

            * offer message will include: taskID, offerVal, pos, startTime, endTime
        */


        // Calculate path
        this.mp.setGoals(this.pos);
        this.mp.setStart(TApos);
        if (!this.mp.plan()) throw new Error ("No path between " + "current_pos" + " and " + TApos);
        PoseSteering[] path = this.mp.getPath();

        double startTime = Double.parseDouble(mParts[3]);
        double endTime = this.calculateEndTime(startTime, path);

        // If task is not possible
        if (this.timeSchedule.taskPossible(startTime, endTime) == false) {
            return false; 
        }

        // SCHEDULE: Create new task & and add it to schedule
        Task DAtask = new Task(Integer.parseInt(mParts[0]), m.sender, false, ore, startTime, endTime, TApos, this.pos);
        this.timeSchedule.add(DAtask);
        // this.timeSchedule.printSchedule();

        // offer value calc
        double distToTA = this.calcDistance(this.pos, TApos);

        int offer = this.evalService(distToTA);

        //distToTA = 100.0 * 1.0 / distToTA;
        //System.out.println(this.robotID + " dist eval --------->" + distToTA );
        //double evaluatedCapacity = 100.0 * this.amount / this.capacity; 

        // generate offer..
        //int offer = (int)(distToTA + evaluatedCapacity);
        Message response = createOffer(m, mParts, TApos, this.pos, offer, startTime, endTime, ore);
        
        //send offer and log event
        this.sendMessage(response);

        //System.out.println(this.robotID + ", task: " + this.activeTasks.get(Integer.parseInt(mParts[0])));
        return true;
    }

    /**
     * Helper function for creating an offer to respond a service. 
     * @param message
     * @param messageParts
     * @param position
     * @param offer
     * @return
     */
    protected Message createOffer(Message message, String[] messageParts, Pose startPose, Pose endPos, int offer, double startTime, double endTime, double ore) {
        String startPoseStr = this.stringifyPose(startPose);
        String endPoseStr = this.stringifyPose(endPos);
        String s = this.separator;
        String body = messageParts[0] +s+ offer +s+ startPoseStr +s+ 
                      endPoseStr +s+ startTime +s+ endTime +s+ ore;
        return new Message(this.robotID, message.sender, "offer", body);
    } 

    protected int evalService(double dist){
        System.out.println(this.robotID + "\teval --------->" + dist );
        if (dist <= 2.0) return -1;

        dist = 100 * 1.0 / dist;
        double evaluatedCapacity = 50.0 * this.amount / this.capacity; 

        return (int)(dist + evaluatedCapacity);
    }
    
}
