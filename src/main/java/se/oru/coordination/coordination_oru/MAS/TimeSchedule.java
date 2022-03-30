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




    public void update(int taskID, double newEndTime){

    }
    public boolean taskPossible(double start, double end){  // return true if possible
        for (int i=0; i<this.schedule.size(); i++){ // case size >1
            Task curr = this.schedule.get(i);

            if ( curr.endTime < start ) continue;

            if (end < curr.startTime ) break;
            else return false;

        }
        return true;
    } 

    public double getNextStartTime(){ //return endTime for last task
        return 0.0;
    } 
    public double checkEndStateOreLvl(){
        return 0.0;
    } // return the last state

    protected boolean add(Task task) {

        if (this.schedule.size() <= 0){                                 // case size = 0
            this.schedule.add(task);
            return true;
        }

        for (int i=0; i<this.schedule.size(); i++){                     // case size > 0
            Task curr = this.schedule.get(i);

            if ( curr.endTime <= task.startTime ){   // if curr is left of task
                if ( i == this.schedule.size()-1){ // task should be added at end of schedule
                    this.schedule.add(task);
                    return true;
                }
                else continue;
            }
            if ( this.isTaskOverlapping(task, curr) ) return false;

            this.schedule.add(i, task);
            return true;            
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

        /*
        if ( s1 > s2 ){     //should cover all cases
            if (e1 < e2) return true;
            if (e2 > s1) return true;
        }
        else {
            if (e1 > e2) return true;
            if (e1 > s2) return true;
        }
        */

        // if: slot1's left is IN slot2 OR slot1's right is IN slot2 OR whole slot2 is in slot1 
        /*
        if ( (s1>s2 && s1<e2) || (e1<e2 && e1>s2) || (s1<s2 && e1>e2)) return true;
        */
        if (e1 > s2 && s1 < e2) return true;
        return false;
    }
        
    

    

    // TODO ABORT TASK
    // TODO REPLACE TASK

    protected int getSize() {
        return this.schedule.size();
    }


    protected void changeTaskOrder() {}

    public void printSchedule(){
        System.out.println("___________________________________SCHEDULE_________________________________________");
        for (Task t : this.schedule){
            System.out.println("time: " + t.startTime + " --> " + t.endTime);
        }

        System.out.println("____________________________________________________________________________________");


    }


    public static void main(String args[]){

        TimeSchedule ts = new TimeSchedule();


        //overlap test
        boolean testOverlap = false;
        boolean testAddFunc = true;

        if (testOverlap){
            System.out.println("######### testing isTaskOverlapping(Task t, Task t) #########");

            Task over1 = new Task(10.0, 20.0);
            Task over2 = new Task(5.0, 15.0);
            
            System.out.println( ts.isTaskOverlapping(over1, over2) == true ? "success" : "fail");

            over2 = new Task(15.0, 25.0);
            System.out.println( ts.isTaskOverlapping(over1, over2) == true ? "success" : "fail");

            over2 = new Task(5.0, 25.0);
            System.out.println( ts.isTaskOverlapping(over1, over2) == true ? "success" : "fail");

            over2 = new Task(12.0, 18.0);
            System.out.println( ts.isTaskOverlapping(over1, over2) == true ? "success" : "fail");

            over2 = new Task(5.0, 10.0);
            System.out.println( ts.isTaskOverlapping(over1, over2) == false ? "success" : "fail");

            over2 = new Task(20.0, 25.0);
            System.out.println( ts.isTaskOverlapping(over1, over2) == false ? "success" : "fail");
        }


        if (testAddFunc){
            System.out.println("######### testing add(Task t) #########");

            Task t1 = new Task(10.0, 20.0);
            Task t2 = new Task(30.0, 40.0);
            Task t3 = new Task(50.0, 60.0);
            Task t4 = new Task(40.0, 50.0);
            Task t5 = new Task(100.0, 110.0);
            Task t6 = new Task(25.0, 35.0);

            System.out.println( ts.add(t1) == true ? "t1 success" : "t1 fail");
            System.out.println( ts.add(t2) == true ? "t2 success" : "t2 fail");
            System.out.println( ts.add(t3) == true ? "t3 success" : "t3 fail");
            System.out.println( ts.add(t4) == true ? "t4 success" : "t4 fail");
            System.out.println( ts.add(t5) == true ? "t5 success" : "t5 fail");
            System.out.println( ts.add(t6) == false ? "t6 success" : "t6 fail");
            System.out.println( ts.add(new Task(25.0, 30.0)) == true ? "t7 success" : "t7 fail");
            System.out.println( ts.add(new Task(5.0, 10.0)) == true ? "t8 success" : "t8 fail");
            System.out.println( ts.add(new Task(109.0, 111.0)) == false ? "t9 success" : "t9 fail");
            ts.printSchedule();

        }








    }
}
