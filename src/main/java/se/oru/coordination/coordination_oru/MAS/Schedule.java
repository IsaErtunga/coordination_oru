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


import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
*/

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class Schedule {

    protected Pose lastToPose;
    protected Task currentTask = null;
    
    // Data structure for storing tasks. 
    private ArrayList<Task> schedule = new ArrayList<Task>();

    protected void enqueue(Task task) {
        // Add to end of schedule queue
        this.lastToPose = task.toPose;
        this.schedule.add(task);
    }

    protected Task dequeue() {
        // Pop schedule queue
        if (this.schedule.size() <= 0) return null;

        this.currentTask = this.schedule.remove(0);
        return this.currentTask;
    
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

    protected boolean isLastTaskSA(){

        if (this.schedule.size()<=0){
            if (this.currentTask != null){
                return this.currentTask.isSATask;
            }
            return false;
        }

        Task task = this.schedule.get(this.schedule.size() - 1);

        return task.isSATask;

    }

    protected void changeTaskOrder() {}

    public void printSchedule(){
        System.out.println("___________________________________SCHEDULE_________________________________________");
        if (this.currentTask != null){
            System.out.println("Current task exists::");
            System.out.println("from: " + this.currentTask.fromPose.toString() + " --> " + this.currentTask.toPose.toString());
            System.out.println("taskprovider: " + this.currentTask.taskProvider);
            System.out.println("\n");
        }
        else{
            System.out.println("Current task is null::");
        }
        for (Task t : this.schedule){
            System.out.println("from: " + t.fromPose.toString() + " --> " + t.toPose.toString());
            System.out.println("taskprovider: " + t.taskProvider);
            System.out.println("\n");
        }

        System.out.println("____________________________________________________________________________________");


    }

}
