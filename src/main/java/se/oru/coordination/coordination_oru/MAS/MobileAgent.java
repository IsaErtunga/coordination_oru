package se.oru.coordination.coordination_oru.MAS;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Collections;
import java.lang.Math;

public class MobileAgent extends AuctioneerBidderAgent{

    protected Coordinate[] rShape;
    protected double robotSpeed = 20.0;
    protected double robotAcceleration = 10.0;
    protected TrajectoryEnvelopeCoordinatorSimulation tec;

    protected String STATE;
    protected Task sCurrTask = null;
    protected Task sNextTask = null;
    protected Mission sCurrMission = null;
    protected Mission sNextMission = null;

    protected boolean startReplan = false;
    protected ArrayList<Pose> waitingWithTaskPoses = new ArrayList<Pose>();

    protected FilePrinter fp;
    protected double robotBreakdownTestProb = 0.0;
    
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

    protected void breakRobotTest(){
        if ( this.robotBreakdownTestProb == 0.0 ) return;

        boolean agentWillBreak = this.rand.nextInt(1000) <= (int)(this.robotBreakdownTestProb*1000);
        if ( !agentWillBreak ) return;

        int secondsBeforeBoom = 2*60 + this.rand.nextInt( this.rand.nextInt(4*60)) - (int)this.getTime();  //TODO fix stupid boyy
        this.sleep(1000 * secondsBeforeBoom);
        PoseSteering[] newPath = this.calculatePath(this.mp, new Pose(139.5, 22.0, Math.PI/2), new Pose(139.5, 22.0, Math.PI/2));

        synchronized(this.timeSchedule){ this.timeSchedule.wipeSchedule(); }
        synchronized(this.inbox){ this.inbox.add(0, new Message()); }
        this.sCurrMission = null;
        this.sCurrTask = null;
        this.sNextMission = null;
        this.sNextTask = null;
        this.STATE = "BREAK";

        synchronized(this.tec){
            this.tec.replacePath(this.robotID, newPath, 0, false, Collections.EMPTY_SET);
        }
        this.sleep(500);
        synchronized(this.inbox){ this.inbox.clear(); }
    }
    
    protected void setRobotSpeedAndAcc(double speed, double acc){
        this.robotSpeed = speed;
        this.robotAcceleration = acc;
        synchronized(this.tec){
            this.tec.setRobotMaxVelocity(this.robotID, speed);
            this.tec.setRobotMaxAcceleration(this.robotID, acc);
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
            }
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
            if ( t.partner == -1 ) continue;
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
                    // if prevState == START_NEXT_MISSION_STATE
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
                case "BREAK":
                    // for testBreakRobot
                    return;
            }
        }
    }

    protected void startNextMissionState(){
        this.prepareNextMissionState();
        if ( sNextMission == null ) return;

        double now = this.getTime();
        double meassureStart = now;
        double stopSleepTime = (sNextTask.startTime - this.getTime()) - 0.5;
        if ( stopSleepTime > 0.0 ){
            this.print("starting to sleep for -->"+stopSleepTime);
            this.sleep( (int)(stopSleepTime*1000) );
            this.print("done sleeping");
        }

        synchronized(this.timeSchedule){this.timeSchedule.getNextEvent();} // here we dedicate to doing the mission

        sCurrMission = sNextMission;                                                   // start next mission
        sCurrTask = sNextTask;
        sNextMission = null;
        sNextTask = null;

        synchronized(this.tec){ this.tec.addMissions(sCurrMission); }
        Double elapsedTime = this.getTime() - meassureStart;
        this.fp.addWaitingTimeMeasurment("idleUponExecution", elapsedTime, this.robotID); 

        this.print("--startNextMissionState: mission added, task startTime-->"+sCurrTask.startTime);

        if ( stopSleepTime < -0.5 ) { // update schedule partners of new delay if we start late
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

        sNextMission = this.createMission(sNextTask, sCurrTask == null ? this.initialPose : sCurrTask.toPose);

        PoseSteering[] path = sNextMission.getPath();
        double pathDist = this.calculatePathDist(path);
        
        if ( pathDist/sNextTask.pathDist > 1.3 || path[path.length-1].getPose().distanceTo(sNextTask.toPose) > 3.0 ){
            this.print(  "--prepareNextMissionState: path calculated but not good. path is "+(pathDist/sNextTask.pathDist)
                        +" times the size of distEst and "+(path[path.length-1].getPose().distanceTo(sNextTask.toPose))+" meters away from goalPose");
            sNextMission = null;
            sNextTask = null;
        } else {
            this.print("--prepareNextMissionState: created mission in-->"+(this.getTime() - now)+" seconds");
        }
    }

    protected void trackMissionState(){
        this.print("--trackMissionState");
        boolean missionIsDone = false;
        boolean isWaiting = false;
        double startTime = 0.0;

        while ( true ){
            this.sleep(500);
            this.prepareNextMissionState(); // replan next mission if it is changed

            synchronized(this.tec){ missionIsDone = this.tec.isFree(this.robotID); } 

            if ( missionIsDone ){ // if we are done with mission
                this.sleep( (int)( this.LOAD_DUMP_TIME*1000.0) );
                this.print("--trackMissionState: task done. task endTime-->"+sCurrTask.endTime);
                this.fp.addDistanceMeasurment("Task", this.calculatePathDist(sCurrMission.getPath()), this.robotID);
                this.sleep((int)this.LOAD_DUMP_TIME*1000);
                if ( sCurrTask.partner != -1 ){
                    Message doneMessage = new Message(this.robotID, sCurrTask.partner, "inform", sCurrTask.taskID + this.separator + "done" + "," + sCurrTask.ore);
                    this.sendMessage(doneMessage);
                } else {
                    this.amountLaps += 1;
                    if (this.amountLaps >= 10) {
                        this.MissionOver.add(0, true);
                    }
                    //this.fp.logCollectedOre(Math.abs(sCurrTask.ore));
                }
                this.STATE = "START_NEXT_MISSION_STATE";
                break; 
            }

            boolean robotWaitingNow = this.isRobotWaiting();
            if ( isWaiting == false && robotWaitingNow == true ){
                startTime = this.getTime();
                this.print("started waiting");
                isWaiting = true;

            } else if ( isWaiting == true && robotWaitingNow == true && this.getTime() - startTime > 5.0 ){
                Double elapsedTime = this.getTime() - startTime;
                this.fp.addWaitingTimeMeasurment("congestion", elapsedTime, this.robotID); 
                startTime = elapsedTime + startTime;
                this.print("waited for 5s");

            } else if ( robotWaitingNow == false && isWaiting == true ){
                Double elapsedTime = this.getTime() - startTime;
                if ( elapsedTime > 1.0 ){
                    this.fp.addWaitingTimeMeasurment("congestion", elapsedTime, this.robotID); 
                    this.print("stopped waiting. elapsed time->"+elapsedTime);
                }
                isWaiting = false;
            }
        }
    }

    protected void replanCurrentMissionState(){
        this.print("--replanCurrentMissionState");

    }

    // /**
    //  * returns if robot is waiting at critical point that is not near end of mission
    //  * @return true if waiting, else false
    //  */
    protected boolean isRobotWaiting(){
        RobotReport rr;
        synchronized(this.tec){ rr = this.tec.getRobotReport(this.robotID); }
        int currPathIndex = rr.getPathIndex();
        int pathLen = sCurrMission.getPath().length;
        return rr.getCriticalPoint() == currPathIndex && pathLen-4 > currPathIndex ;
    }
}
