/**
 * @author Alexander Benteby & Isa Ertunga
 * This class is representing a task that a TA will execute
 */

package se.oru.coordination.coordination_oru.MAS;

import se.oru.coordination.coordination_oru.Mission;

public class Task {
    int taskID;
    Mission mission;
    float expiryTime;
    int taskProvider;
    String status;

    Task() {}

    Task(int taskID, Mission mission, float expiryTime, int taskProvider, String status) {
        this.taskID = taskID;
        this.mission = mission;
        this.expiryTime = expiryTime;
        this.taskProvider = taskProvider;
        this.status = status;
    }

    // Getter & setter.
}
