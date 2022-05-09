package se.oru.coordination.coordination_oru.MAS;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Collections;
import java.lang.Math;

public class MobileAgent extends AuctioneerBidderAgent{

    protected TrajectoryEnvelopeCoordinatorSimulation tec;
    protected Coordinate[] rShape;
    protected double robotSpeed = 20.0;
    protected double robotAcceleration = 10.0;
    protected int TASK_EXECUTION_PERIOD_MS = 200;
    protected Task sCurrTask = null;
    protected Task sNextTask = null;
    protected Mission sCurrMission = null;
    protected Mission sNextMission = null;

    protected boolean startReplan = false;
    protected String STATE;
    protected ArrayList<Pose> waitingWithTaskPoses = new ArrayList<Pose>();
    
    public void addRobotToSimulation(){
        synchronized(this.tec){
        this.tec.setForwardModel(this.robotID, new ConstantAccelerationForwardModel(
            this.robotAcceleration, 
            this.robotSpeed, 
            this.tec.getTemporalResolution(), 
            this.tec.getControlPeriod(), 
            this.tec.getRobotTrackingPeriodInMillis(this.robotID)));

        this.tec.setFootprint(this.robotID, this.rShape);

        this.tec.placeRobot(this.robotID, this.initialPose);
        
        this.tec.setMotionPlanner(this.robotID, this.mp); // Motion planner
        }
    }
    
    protected void setRobotSpeedAndAcc(double speed, double acc){
        this.robotSpeed = speed;
        this.robotAcceleration = acc;
        synchronized(this.tec){
            this.tec.setRobotMaxVelocity(this.robotID, speed);
            this.tec.setRobotMaxAcceleration(this.robotID, acc);
        }
    }

    protected void taskExecutionThread(){ // basic task execution function
        while (true) {
            this.sleep(TASK_EXECUTION_PERIOD_MS);

            Task task = null;
            Pose prevToPose = null;
            synchronized(this.timeSchedule){
                prevToPose = this.timeSchedule.getPoseAtTime(-1.0);
                task = this.timeSchedule.getNextEvent();
            }

            if (task == null) continue;

            this.print("starting mission taskID-->"+task.taskID+" with -->" +task.partner + "\tat time-->"+this.getTime()+"\ttaskStartTime-->"+task.startTime);
            synchronized(this.tec){ this.tec.addMissions(this.createMission(task, prevToPose)); }

            this.waitUntilCurrentTaskComplete(100); // locking
            this.sleep((int)this.LOAD_DUMP_TIME*1000);

            this.print("mission DONE taskID-->"+task.taskID+" with -->" +task.partner + "\tat time-->"+this.getTime()+"\ttaskEndTime-->"+task.endTime);
            if ( task.partner != -1 ){
                Message doneMessage = new Message(this.robotID, task.partner, "inform", task.taskID + this.separator + "done" +this.separator+ (-task.ore));
                this.sendMessage(doneMessage);
            }
            
        }
    }

    /**
     * 
     * @param startPose
     * @param endPose
     * @return
     */
    public Mission createMission(Task task, Pose prevToPose) {
        return new Mission(this.robotID, this.calculatePath( this.mp, prevToPose, task.toPose ));
    }

    /**
     * Compare robot pose to task end pose to see if Task is finished
     * @param task
     * @return
     */
    protected double waitUntilCurrentTaskComplete (int cycleSleepTimeMs) {
        double now = this.getTime();

        //TODO if in deadlock and in later stage of mission
        /* functions in tec
        public boolean replanEnvelope(int robotID)
        protected boolean inParkingPose(int robotID)
        */
        while ( true ){
            synchronized(this.tec){
                if ( this.tec.isFree(this.robotID) == true ) break;
                //rr = this.tec.getRobotReport(this.robotID);   
            }
            // if ( rr.getCriticalPoint() == rr.getPathIndex() ){
            //     // robot is waiting
            // } else {
            //     // robot is moving
            // }

            this.sleep(cycleSleepTimeMs);
        }
        return this.getTime() - now;
    }

    /**
     * function used to send new task times to 
     * @param tasksToUpdate
     * @param isLater
     */
    protected void sendInformStatusMessages(ArrayList<Task> tasksToUpdate, boolean isLater){
        if ( isLater ) Collections.reverse(tasksToUpdate);
        String updateSep = "::";
        String pairSep = ":";

        HashMap<Integer, ArrayList<Task>> taskMap = new HashMap<Integer, ArrayList<Task>>();
        
        for ( Task t : tasksToUpdate ){
            ArrayList<Task> agentTasks = taskMap.get(t.partner);
            if ( agentTasks == null ) agentTasks = new ArrayList<Task>();
                
            agentTasks.add(t);
            taskMap.put(t.partner, agentTasks);
        }

        for (int key : taskMap.keySet()) {
            ArrayList<Task> tasksNewTime = taskMap.get(key);
            Task firstElement = tasksNewTime.remove(0);

            String messageBody = "0" +this.separator+ "status" +this.separator+ firstElement.taskID + pairSep + firstElement.endTime;

            for ( Task t : tasksNewTime ){
                messageBody = messageBody +updateSep+ t.taskID +pairSep+ t.endTime;
            }
            this.sendMessage(new Message(this.robotID, key, "inform", messageBody));
        }
    }

    protected void stateHandler(){
        this.STATE = "START_NEXT_MISSION_STATE";

        while ( true ){
            this.sleep(100);
            switch(this.STATE){
                case "START_NEXT_MISSION_STATE":
                    this.startNextMissionState();
                    break;
    
                case "TRACK_MISSION_STATE":
                    this.trackMissionState();

                    // make sure we have no deadlocks and stuff
                    break;
                case "PREPARE_MEXT_MISSION_STATE":
                    this.prepareNextMissionState();
                    // make sure we have no deadlocks and stuff
                    break;
                case "REPLAN_MISSION_STATE":
                    this.replanCurrentMissionState();
                    // make sure we have no deadlocks and stuff
                    break;
            }
        }
    }

    protected void startNextMissionState(){
        this.prepareNextMissionState();
        if ( sNextTask == null ) return;

        double now = this.getTime();
        double timeUntilMissionStart = sNextTask.startTime - now;    // handle if next mission starts later
        this.print("--startNextMissionState: time until mission starts: "+timeUntilMissionStart);
        if ( timeUntilMissionStart > 4.0 ){
            this.sleep( 1000 );
            return;

        } else if ( timeUntilMissionStart > 0.5 ){
            this.sleep( (int)((timeUntilMissionStart-0.5)*1000.0) );
        }
        synchronized(this.timeSchedule){this.timeSchedule.getNextEvent();} // here we dedicate to doing the mission

        sCurrMission = sNextMission;                                                   // start next mission
        sCurrTask = sNextTask;
        sNextMission = null;
        sNextTask = null;
        synchronized(this.tec){ this.tec.addMissions(sCurrMission); }
        this.print("--startNextMissionState: added mission to tec");

        if ( timeUntilMissionStart < -0.5 ) { // update schedule partners of new delay if we start late
            double nextStartTime = sCurrTask.endTime - sCurrTask.startTime + now;
            sCurrTask.startTime = now;
            sCurrTask.endTime = nextStartTime;
            ArrayList<Task> newEndTimes;
            synchronized(this.timeSchedule){
                newEndTimes = this.timeSchedule.update(nextStartTime);
                this.timeSchedule.changeOreStateEndTime(sCurrTask.taskID, nextStartTime);
            }
            newEndTimes.add(0, sCurrTask);
            this.sendInformStatusMessages( newEndTimes, true );  
            this.print("--startNextMissionState: sent inform status msgs");
        }

        this.STATE = "TRACK_MISSION_STATE";
    }

    protected void prepareNextMissionState(){
        Task nextTinSchedule;
        synchronized(this.timeSchedule){ nextTinSchedule = this.timeSchedule.getNextTask(); }
        if ( sNextTask != null && nextTinSchedule != null && nextTinSchedule.taskID == sNextTask.taskID ) return;
        if ( nextTinSchedule == null ) return;
        
        sNextTask = nextTinSchedule;
        double now = this.getTime();

        double planTime = 5.0;

        this.mp.setPlanningTimeInSecs(planTime);
        sNextMission = this.createMission(sNextTask, sCurrTask == null ? this.initialPose : sCurrTask.toPose);
        this.print("--prepareNextMissionState: created mission in-->"+(this.getTime() - now)+" seconds");
    }

    protected void trackMissionState(){
        this.print("--trackMissionState");
        int scSize;
        boolean missionIsDone = false;
        boolean isWaiting = false;
        RobotReport rr;

        while ( true ){
            this.sleep(1000);
            this.prepareNextMissionState(); // replan next mission if it is changed

            // if ( isWaiting && this.startReplan){
            //     this.STATE = "REPLAN_CURRENT_MISSION";
            //     break;
            // }

            synchronized(this.tec){ 
                missionIsDone = this.tec.isFree(this.robotID);
                rr = this.tec.getRobotReport(this.robotID); 
            } 

            if ( missionIsDone ){ // if we are done with mission
                this.sleep((int)this.LOAD_DUMP_TIME*1000);
                Message doneMessage = new Message(this.robotID, sCurrTask.partner, "inform", sCurrTask.taskID + this.separator + "done" + "," + sCurrTask.ore);
                this.sendMessage(doneMessage);
                this.STATE = "START_NEXT_MISSION_STATE";
                break; 
            }

            if ( isWaiting == false && rr.getCriticalPoint() == rr.getPathIndex()){
                this.print("started waiting");
                isWaiting = true;
                continue;
            }
            isWaiting = rr.getCriticalPoint() == rr.getPathIndex();

            // if ( isWaiting == false && rr.getCriticalPoint() == rr.getPathIndex()){ // if we notice now that we are waiting for path to slove
            //     isWaiting = true;
            //     this.waitingWithTaskPoses.add(0, currentT.fromPose);
            //     this.waitingWithTaskPoses.add(1, currentT.toPose);
            //     String body = this.stringifyPose(currentT.fromPose)+this.separator+this.stringifyPose(currentT.toPose);
            //     this.sendMessage(new Message(this.robotID, this.getReceivers("TRANSPORT"),"waiting",body), true);
            // }
        }
    }

    protected void replanCurrentMissionState(){
        this.print("--replanCurrentMissionState");

    }



}
