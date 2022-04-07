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

        this.timeSchedule = new TimeSchedule(pos, capacity);
        this.startTime = startTime;

        router.enterNetwork(this.robotID, this.inbox, this.outbox);
        this.sendMessage(new Message(this.robotID, "hello-world", ""), true);

    }

    protected double getTime(){
        long diff = System.currentTimeMillis() - this.startTime;
        return (double)(diff)/1000.0;
    }

    public void takeOre(double oreChange){
        oreChange = oreChange < 0.0 ? oreChange : -oreChange; // make sure sign is negative
        this.amount += oreChange;

        double x = this.finalXPos + (this.initalXPos - this.finalXPos) * this.amount / this.capacity;
        this.pos = new Pose( x, pos.getY(), pos.getYaw() );

        this.print("position updated"+this.pos.toString());
    }


    public void listener(){
        ArrayList<Message> inbox_copy;

        while(true){
        
            synchronized(inbox){
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

                else if (m.type == "accept"){
                    if ( !this.timeSchedule.setTaskActive(taskID) ){ 
                        System.out.println(this.robotID +"\t got 'accept' but cant add task to schedule.");
                    }
                    else{
                        System.out.println(this.robotID +"\t TASK ADDED. updated schedule:");
                        this.timeSchedule.printSchedule();
                    }
                }

                else if (m.type == "decline"){} //TODO remove task from reservedTasks in schedule

                else if (m.type == "cnp-service"){
                    this.handleService(m);
                }

                else if (m.type.equals(new String("inform"))) {
                    // TA informs SA when its done with a task.
                    String[] messageParts = this.parseMessage(m, "", true);
                    String informVal = messageParts[1];
                    
                    if (informVal.equals(new String("done"))) {
                        double oreChange = Double.parseDouble(messageParts[2]); 
                        this.timeSchedule.remove(taskID);
                        this.takeOre(oreChange);
                    }

                    else if (informVal.equals(new String("status"))) {} //TODO change so schedule gets updated: newEndTime = Double.parseDouble(messageParts[2])                    

                    else if (informVal.equals(new String("abort"))) {} //TODO remove task from schedule 

                }
                
            }

            try { Thread.sleep(1000); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
    }


    public PoseSteering[] calculatePath(Pose from, Pose to){
        this.mp.setGoals(from);
        this.mp.setStart(to);
        if (!this.mp.plan()) throw new Error ("No path between " + from + " and " + to);

        return this.mp.getPath();
    }


    /**
     * DA responds to TA with offer that is calculated in this function. 
     * SCHEDULE:
     * - Will receive a time from TA of when it can come and fetch ore. 
     */
    public boolean handleService(Message m){ 

        this.print("aaa-start");

        double availabeOre = this.timeSchedule.checkEndStateOreLvl();
        if (availabeOre <= 0.0) return false;   //if we dont have ore dont act 

        else availabeOre = availabeOre >= 15.0 ? 15.0 : availabeOre; // only give what ore we have available

        Task DAtask = createTaskFromServiceOffer(m, availabeOre);

        if ( !this.timeSchedule.taskPossible(DAtask) ) return false;    // task doesnt fit in schedule

        int offerVal = this.calculateOffer(DAtask);

        if ( offerVal <= 0 ) return false;
        
        if (! this.timeSchedule.add(DAtask) ){
            this.print("not added!");
            return false;
        }
        
        /*
        if( offerVal <= 0 && this.timeSchedule.add(DAtask) == false ) return false; 
        offerVal == 0:                      enter IF
        offerVal > 0 -> add(Task) == true:  dont enter IF 
        offerVal > 0 -> add(Task) == false: enter IF 
        */
        this.timeSchedule.printSchedule();

        this.print("aaa-final");

        this.sendMessage(this.createOfferMsgFromTask(DAtask, offerVal, availabeOre));

        return true;
    }

    protected Message createOfferMsgFromTask(Task t, int offer, double ore){
        String s = this.separator;

        String startPoseStr = this.stringifyPose(t.fromPose);
        String endPoseStr = this.stringifyPose(t.toPose);
        String body = t.taskID +s+ offer +s+ startPoseStr +s+ 
                      endPoseStr +s+ startTime +s+ t.endTime +s+ ore;

        return new Message(this.robotID, t.partner, "offer", body);
    }


    protected Task createTaskFromServiceOffer(Message m, double ore){
        String[] mParts = this.parseMessage(m, "", true);

        Pose TApos = this.posefyString(mParts[2]);
        PoseSteering[] path = this.calculatePath(this.pos, TApos);
        double distToTA = this.calcDistance(this.pos, TApos);

        double startTime = Double.parseDouble(mParts[3]);
        double endTime = this.calculateEndTime(startTime, path);

        return new Task(Integer.parseInt(mParts[0]), m.sender, false, -ore, startTime, endTime, distToTA, TApos, this.pos);
    }


    protected int calculateOffer(Task t){
        if (t.pathDist <= 2.0) return 0;

        double dist = 100.0 * 1.0 / t.pathDist;
        double evaluatedCapacity = 50.0 * this.amount / this.capacity; 

        System.out.println("aaaa");



        return (int)(dist + evaluatedCapacity);
    }

    protected void print(String s){
        System.out.println("\033[0;36m"+this.robotID+"\t" + s + "\033[0m");
    }
    
}
