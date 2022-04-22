package se.oru.coordination.coordination_oru.MAS;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.Mission;

public class MobileAgent extends BasicAgent{

    protected TrajectoryEnvelopeCoordinatorSimulation tec;
    protected Coordinate[] rShape;
    protected Pose startPose;
    protected double robotSpeed;
    protected double robotAcceleration;
    
    /**
     * 
     */
    public void addRobotToSimulation(){
        synchronized(this.tec){
        this.tec.setForwardModel(this.robotID, new ConstantAccelerationForwardModel(
            this.robotAcceleration, 
            this.robotSpeed, 
            this.tec.getTemporalResolution(), 
            this.tec.getControlPeriod(), 
            this.tec.getRobotTrackingPeriodInMillis(this.robotID)));

        this.tec.setFootprint(this.robotID, this.rShape);

        this.tec.placeRobot(this.robotID, this.startPose);
        
        tec.setMotionPlanner(this.robotID, this.mp); // Motion planner
        }
    }

    protected void setRobotSize(double xLength, double yLength){
        this.rShape = new Coordinate[] {new Coordinate(-xLength,yLength),new Coordinate(xLength,yLength),
                                        new Coordinate(xLength,-yLength),new Coordinate(-xLength,-yLength)};
    }

    protected void setSchedule(){
        this.timeSchedule = new TimeScheduleNew(this.startPose, this.capacity, this.initialOreAmount);
    }

    protected void taskExecutionThread(){ // basic task execution function
        while (true) {
            this.sleep(200);

            Task task = null;
            Pose prevToPose = null;
            synchronized(this.timeSchedule){
                prevToPose = this.timeSchedule.getPoseAtTime(-1.0);
                task = this.timeSchedule.getNextEvent();
            }

            if (task == null) continue;

            synchronized(this.tec){ this.tec.addMissions(this.createMission(task, prevToPose)); }

            this.waitUntilCurrentTaskComplete(this.robotID, 100); // locking

            Message doneMessage = new Message(this.robotID, task.partner, "inform", task.taskID + this.separator + "done" +this.separator+ (-task.ore));
            this.sendMessage(doneMessage);
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
    protected void waitUntilCurrentTaskComplete (int agentID, int cycleSleepTimeMs) {
        while ( true ){
            synchronized(tec){
                if ( tec.isFree(agentID) == true ) break;
            }
            this.sleep(cycleSleepTimeMs);
        }
    }

}
