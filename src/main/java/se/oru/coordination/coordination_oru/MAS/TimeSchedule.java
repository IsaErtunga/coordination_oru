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

/* Schedule:
    * add(Task task) -> bool
    * remove(int taskID) -> Task
    * get(int taskID) --> Task
    * update(int taskID, double newEndTime) --> bool
    * taskPossible(double startTime, double endTime) --> bool // check if timeslot is possible
    * getNextStartTime() --> double startTime
    
    (oreState will be updated internally but agents can check it via schedule)
    * this.oreState arrayList<double time, double oreLvl>
    * checkEndStateOreLvl() -> double oreLvl

    Isa
    * func som kollar när den kan börja ett nytt mission
    getNextStartTime() -> double startTime
*/

public class TimeSchedule {

    protected Task currentTask = null;    
    private ArrayList<Task> schedule = new ArrayList<Task>();
    private HashMap<Double, Integer> state = new HashMap<Double, Integer>();

    public void update(int taskID, double newEndTime){}
    public boolean taskPossible(double start, double end){return false;} // return true if possible
    public double getNextStartTime(){return 0.0;} //return endTime for last task
    public double checkEndStateOreLvl(){return 0.0;} // return the last state

    protected boolean add(Task task) {
        int index = 0;

        if (this.schedule.size() <= 0){ // case size=0
            this.schedule.add(task);
            return true;
        }

        else if (this.schedule.size() == 1){    // case size=1 
            if ( this.isTaskOverlapping(this.schedule.get(0), task) ) return false; // not added 
            
            if (this.schedule.get(0).startTime < task.startTime) index = 1;
            else index = 0;

            this.schedule.add(index, task);
            return true;
        } 

        for (int i=0; i<this.schedule.size(); i++){ // case size >1
 
            this.schedule.get(i);
        }
        return false;
    }

    protected Task remove(int taskID) {
        return this.schedule.remove(taskID);
    }

    protected Task get(int taskID) {
        // I get task you get taskID
        for (Task task : this.schedule) {
            if (task.taskID == taskID) {
                return task;
            }
        }
        return null;

    }

    private boolean isTaskOverlapping(Task t1, Task t2){        // a1 is start of slot1, b1 is end of slot1
        double s1 = t1.startTime;       // a1 > a2 & b1 > b2 & b2 > a1  = slot2 is semi to the left of slot1
        double e1 = t1.endTime;         // a1 < a2 & b1 < b2 & b1 > a2  = slot2 is semi to the right of slot1
        double s2 = t2.startTime;       // a1 < a2 & b1 > b2            = slot2 is completly within slot1
        double e2 = t2.endTime;         // a1 > a2 & b1 < b2            = slot1 is completly within slot2

        if ( s1 > s2 ){     //should cover all cases
            if (e1 < e2) return true;
            if (e2 > s1) return true;
        }
        else {
            if (e1 > e2) return true;
            if (e1 > s2) return true;
        }

        return false;
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
   
    

}
