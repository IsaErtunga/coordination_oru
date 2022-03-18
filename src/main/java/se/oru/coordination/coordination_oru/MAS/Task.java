/**
 * @author Alexander Benteby & Isa Ertunga
 * This class is representing a task that a TA will execute
 */

package se.oru.coordination.coordination_oru.MAS;

import se.oru.coordination.coordination_oru.Mission;

public class Task {
    int taskID;
    float expiryTime;
    String status;

    // TA
    Mission mission;
    int taskProvider;

    // SA
    double ore;


    Task(int taskID, Mission mission, float expiryTime, int taskProvider, String status) {
        // Constructor for TA
        this.taskID = taskID;
        this.mission = mission;
        this.expiryTime = expiryTime;
        this.taskProvider = taskProvider;
        this.status = status;
    }


    Task(int taskID, float expiryTime, String status, double ore) {
        // Constructor for SA
        this.taskID = taskID;
        this.expiryTime = expiryTime;
        this.status = status;
        this.ore = ore;
    }

    // Getter & setter.
}
