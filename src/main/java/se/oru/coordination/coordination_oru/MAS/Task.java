/**
 * @author Alexander Benteby & Isa Ertunga
 * This class is representing a task that a TA will execute
 */

package se.oru.coordination.coordination_oru.MAS;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.Mission;

public class Task {
    int taskID;
    double startTime;
    double endTime;
    boolean isActive;
    double pathDist;
    int partner;
    Pose fromPose;
    Pose toPose;

    // cascading TA
    Task taskDA = null;
    Task taskSA = null;


    // SA
    double ore;

    Task(int taskID, float expiryTime, String status, double ore) {
        // Constructor for SA
        this.taskID = taskID;
        this.ore = ore;
    }

    /**
     * Constructor in use.
     * @param taskID
     * @param partner
     * @param mission
     * @param isActive
     * @param ore
     * @param startTime
     * @param endTime
     * @param fromPose
     * @param toPose
     */
    Task(int taskID, int partner, boolean isActive, double ore, double startTime, double endTime, Pose fromPose, Pose toPose) {
        this.taskID = taskID;
        this.partner = partner;
        this.isActive = isActive;
        this.ore = ore;
        this.startTime = startTime;
        this.endTime = endTime;
        this.fromPose = fromPose;
        this.toPose = toPose;
    }

    Task(int taskID, int partner, boolean isActive, double ore, double startTime, double endTime, double dist, Pose fromPose, Pose toPose) {
        this.taskID = taskID;
        this.partner = partner;
        this.isActive = isActive;
        this.ore = ore;
        this.startTime = startTime;
        this.endTime = endTime;
        this.fromPose = fromPose;
        this.toPose = toPose;
        this.pathDist = dist;
    }


    // ================ for tests ================
    Task(double start, double end){
        this(start, end, 0);
    }

    Task(double start, double end, int taskID){
        this.startTime = start;
        this.endTime = end;
        this.taskID = taskID;
        this.isActive = true;
    }
    // ===========================================

    public void printTask() {
        String sep = ", ";
        String taskID = "taskID: "+this.taskID + sep;
        String partner = "partner: "+this.partner + sep;
        String isActive = "isActive: "+this.isActive + sep;
        String ore = "ore: "+this.ore + sep;
        String startTime = "startTime: "+this.startTime + sep;
        String endTime = "endTime: "+this.endTime + sep;
        String fromPose = "fromPose: "+this.fromPose + sep;
        String toPose = "toPose: "+this.toPose + sep;
        String pathDist = "pathDist: "+this.pathDist + sep;

        System.out.println(taskID+partner+isActive+ore+startTime+endTime+fromPose+toPose+pathDist);

    }
}
