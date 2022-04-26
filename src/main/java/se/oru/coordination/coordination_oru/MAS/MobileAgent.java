package se.oru.coordination.coordination_oru.MAS;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.Mission;

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
        
        tec.setMotionPlanner(this.robotID, this.mp); // Motion planner
        }
    }

    protected void setRobotSize(double xLength, double yLength){
        this.rShape = new Coordinate[] {new Coordinate(-xLength,yLength),new Coordinate(xLength,yLength),
                                        new Coordinate(xLength,-yLength),new Coordinate(-xLength,-yLength)};
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
    protected void waitUntilCurrentTaskComplete (int cycleSleepTimeMs) {
        while ( true ){
            synchronized(this.tec){
                if ( this.tec.isFree(this.robotID) == true ) break;
            }
            this.sleep(cycleSleepTimeMs);
        }
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

}
