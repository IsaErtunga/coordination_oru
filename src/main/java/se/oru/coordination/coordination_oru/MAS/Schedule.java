/**
 * @author Alexander Benteby & Isa Ertunga
 * Schedule imbedded in robot agents for coordinating task execution. 
 */

package se.oru.coordination.coordination_oru.MAS;
import java.util.ArrayList;
import java.util.HashMap;
/*
import java.util.Calendar;
import java.util.Comparator;


import org.sat4j.ExitCode;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import java.util.ArrayList;
import java.util.Arrays;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
*/

public class Schedule {
    
    // Data structure for storing tasks. 
    private ArrayList<Task> schedule = new ArrayList<Task>();

    protected void enqueue(Task task) {
        // Add to end of schedule queue
        this.schedule.add(task);
    }

    protected Task dequeue() {
        // Pop schedule queue
        if (this.schedule.size() > 0) {
            return this.schedule.remove(0);
        }
        return null;
    }

    protected Task getTask(int taskID) {
        // I get task you get taskID
        for (Task task : this.schedule) {
            if (task.taskID == taskID) {
                return task;
            }
        }
        return null;

    }

    // TODO ABORT TASK
    // TODO REPLACE TASK

    protected int getSize() {
        return this.schedule.size();
    }

    protected void changeTaskOrder() {}

}
