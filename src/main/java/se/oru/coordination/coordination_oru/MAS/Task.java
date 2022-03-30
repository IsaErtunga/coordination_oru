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

    

    double startTime;
    double endTime;
    boolean isActive;

    // TA
    Mission mission;
    int taskProvider;
    Pose fromPose;
    Pose toPose;

    // SA
    double ore;


    Task(int taskID, Mission mission, float expiryTime, int taskProvider, String status, Pose fromPose, Pose toPose, boolean isSATask) {
        // Constructor for TA
        this.isSATask = isSATask;
        this.taskID = taskID;
        this.mission = mission;
        this.expiryTime = expiryTime;
        this.taskProvider = taskProvider;
        this.status = status;
        this.fromPose = fromPose;
        this.toPose = toPose;
    }


    Task(int taskID, float expiryTime, String status, double ore) {
        // Constructor for SA
        this.taskID = taskID;
        this.expiryTime = expiryTime;
        this.status = status;
        this.ore = ore;
    }

    Task(int taskID, boolean isActive, double ore, double startTime, double endTime) {
        this.taskID = taskID;
        this.isActive = isActive;
        this.ore = ore;
        this.startTime = startTime;
        this.endTime = endTime;
    }

    // Getter & setter.
}
