/**
 * @author Alexander Benteby & Isa Ertunga
 * This class is representing a task that a TA will execute
 */

package se.oru.coordination.coordination_oru.MAS;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.Mission;

public class Task {
    int taskID;
    float expiryTime;
    String status;
    boolean isSATask;

    

    double startTime;
    double endTime;
    boolean isActive;

    // TA
    Mission mission;
    int partner;
    Pose fromPose;
    Pose toPose;

    // SA
    double ore;


    Task(int taskID, float expiryTime, String status, double ore) {
        // Constructor for SA
        this.taskID = taskID;
        this.expiryTime = expiryTime;
        this.status = status;
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
    Task(int taskID, int partner, Mission mission, boolean isActive, double ore, double startTime, double endTime, Pose fromPose, Pose toPose) {
        this.taskID = taskID;
        this.partner = partner;
        this.mission = mission;
        this.isActive = isActive;
        this.ore = ore;
        this.startTime = startTime;
        this.endTime = endTime;
        this.fromPose = fromPose;
        this.toPose = toPose;
    }

    Task(double start, double end){
        this(start, end, 0);
    }

    Task(double start, double end, int taskID){
        this.startTime = start;
        this.endTime = end;
        this.taskID = taskID;
        this.isActive = true;
    }


    // Getter & setter.
}
