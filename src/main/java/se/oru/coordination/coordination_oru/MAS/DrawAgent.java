package se.oru.coordination.coordination_oru.MAS;
/*
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;

import org.sat4j.ExitCode;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.MAS.Router;
*/
import java.util.ArrayList;
import java.util.Arrays;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.MAS.Router;
import se.oru.coordination.coordination_oru.MAS.Schedule;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.Mission;

public class DrawAgent extends CommunicationAid{

    private final static double finalXPos = 4.0;
    protected double initalXPos;

    protected Pose pos;
    protected double amount;
    protected double capacity; 
    protected Schedule schedule;
    protected ReedsSheppCarPlanner mp;

    protected TimeSchedule timeSchedule;

    public DrawAgent(int robotID, Router router, double capacity, Pose pos, ReedsSheppCarPlanner mp){
        this.robotID = robotID; // drawID >10'000
        this.capacity = capacity;
        this.amount = capacity; // 100% full in beginning
        this.mp = mp;
        this.pos = pos;
        this.initalXPos = pos.getX();

        this.schedule = new Schedule();
        this.timeSchedule = new TimeSchedule();

        router.enterNetwork(this.robotID, this.inbox, this.outbox);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);

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
                if (m.type == "hello-world"){
                    this.robotsInNetwork.add(m.sender);
                    this.sendMessage(
                        new Message( m.receiver.get(0), m.sender, "accept", m.body));
                } 

                else if (m.type == "accept"){
                    this.taskHandler(Integer.parseInt(m.body), m);
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
                    int taskID = Integer.parseInt(messageParts[0]); 
                    String informVal = messageParts[1];
                    
                    if (informVal.equals(new String("done"))) {
                        /* SCHEDULE: 
                            * If early just remove from schedule.
                        */ 
                        int oreChange = Integer.parseInt(messageParts[2]); 
                        this.timeSchedule.remove(this.timeSchedule.get(taskID));
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
    @Override
    public boolean handleService(Message m){ 
        if (m.type != "cnp-service") return false;

        // SCHEDULE: Need time in the message from TA, need to extend with ore. 
        String[] mParts = this.parseMessage(m, "", true); //parse=[ taskID, agentID, pos, startTime ]
        double ore = 10.0;

        // get pose of TA
        
        double[] coordinates = Arrays.stream(mParts[2].split(" ")).mapToDouble(Double::parseDouble).toArray();
        System.out.println(Arrays.toString(coordinates));
        //calc euclidean dist between DA -> TA, and capacity evaluation
        // this.mp.setGoals(goal);
        //     if (!this.mp.plan()) throw new Error ("No path between " + "current_pos" + " and " + goal);
        // PoseSteering[] path = this.mp.getPath();

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

        double startTime = Double.parseDouble(mParts[3]);

        // Calculated time for path TODO change to real calculationt
        double estimatedPathTime = 5.0;

        double endTime = startTime + estimatedPathTime;

        // If task is not possible
        if (this.timeSchedule.taskPossible(startTime, endTime) == false) {
            return false; 
        }

        // SCHEDULE: Create new task & and add it to schedule
        Task DAtask = new Task(Integer.parseInt(mParts[0]), false, ore, startTime, endTime);
        this.timeSchedule.add(DAtask);

        // offer value calc
        double evaluatedDistance = this.calcDistance(this.pos, new Pose(coordinates[0], coordinates[1], coordinates[2]));

        //TODO temp fix
        if (evaluatedDistance <= 0.0) {
            evaluatedDistance = 150.0;
        } 

        evaluatedDistance = 100.0 * 1.0 / evaluatedDistance;
        System.out.println(this.robotID + " dist eval --------->" + evaluatedDistance );
        double evaluatedCapacity = 100.0 * this.amount / this.capacity; 

        // generate offer..
        int offer = (int)(evaluatedDistance + evaluatedCapacity);
        Message response = createOffer(m, mParts, this.pos, offer, startTime, endTime);
        
        //send offer and log event
        this.sendMessage(response);
        this.logTask(Integer.parseInt(mParts[0]),
            "offer" + this.separator + m.sender + this.separator + mParts[2] );
        
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
    protected Message createOffer(Message message, String[] messageParts, Pose position, int offer, double startTime, double endTime) {
        String positionStr = this.stringifyPose(position);
        String body = messageParts[0] + this.separator + offer + this.separator + 
                      positionStr + this.separator + startTime + this.separator + endTime;
        return new Message(this.robotID, message.sender, "offer", body);
    } 
    
}
