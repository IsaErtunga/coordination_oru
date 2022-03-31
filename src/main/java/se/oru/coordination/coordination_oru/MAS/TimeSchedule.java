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

    private static double time_sensitivity = 5.0;
    protected Task currentTask = null;    
    private ArrayList<Task> schedule = new ArrayList<Task>();
    private HashMap<Double, Integer> state = new HashMap<Double, Integer>();




    public HashMap<Integer, Double> update(int taskID, double newEndTime){   //TODO DONT TEST! NOT DONE! WORK IN PROGRESS.
        HashMap<Integer, Double> ret = new HashMap<Integer, Double>();  // add [taskID, newEndTime] if task updated with diff> 5s

        if (this.schedule.size() ==1){ 
            Task t = this.schedule.get(0);
            t.startTime = newEndTime -(t.endTime - t.startTime);
            t.endTime = newEndTime;
            return ret;
        } 

        for ( int i=0; i<this.schedule.size(); i++ ){      
            Task curr = this.schedule.get(i);

            if ( curr.taskID == taskID ){   // found the task being updated
                curr.startTime = newEndTime -(curr.endTime - curr.startTime);
                curr.endTime = newEndTime;

                if ( i != this.schedule.size()-1 && this.isTaskOverlapping(new Task(curr.startTime, newEndTime), this.schedule.get(i+1))){ 
                // if new update generates overlap in tasks after

                    double diff = this.schedule.get(i+1).startTime - curr.endTime;
                    if ( diff > this.time_sensitivity ){
                        ret.put(curr.taskID, curr.endTime);
                    }

                    for ( int j=i+1; j<this.schedule.size(); j++ ){ // for every task that may be affected by overlap
                        curr = this.schedule.get(j);
                                        
                        if (this.schedule.get(j-1).endTime - curr.startTime > this.time_sensitivity){ // if schedule 
                            diff = this.schedule.get(j-1).endTime - curr.startTime;
                            curr.startTime += diff;
                            curr.endTime += diff;
                            ret.put(curr.taskID, curr.endTime);
                        }
                    }
                }
                break;
            }
        }
         
        return ret;
    }

    public HashMap<Integer, Double> update2(int taskID, double newEndTime){
        HashMap<Integer, Double> ret = new HashMap<Integer, Double>();
        int i;

        // find index of task being update and undate.
        for ( i=0; i<this.schedule.size()-1; i++ ){
            Task t = this.schedule.get(i);
            if ( t.taskID == taskID ){
                t.endTime = newEndTime;
                break;
            }
        }

        // check if later schedule needs to be updated

        for ( int j=i; j<this.schedule.size()-1; j++ ){
            Task curr = this.schedule.get(j);
            
            if (curr.endTime - this.schedule.get(j+1).startTime > 5.0){
                System.out.println("big diff");
            }
        }

        return ret;
    }


    /** taskPossible used to check if a slot is available given startTime and endTime.
     * @param start startTIme of task
     * @param end   endTime of task
     * @return  true if slot is available, false if not.
     */
    public boolean taskPossible(double start, double end){  // return true if possible
        Task task = new Task(start, end);
        for (int i=0; i<this.schedule.size(); i++){
            if ( this.isTaskOverlapping(this.schedule.get(i), task) ) return false;
        } 
            
        return true;
    } 


    /** getNextStartTime() will retrive the next time available for scheduling. 
     * return type is double
     * @return the endTime of the last Task in schedule. it does NOT care if task is reserved or not.
     */
    public double getNextStartTime(){ //return endTime for last task
        return this.schedule.get(this.schedule.size()-1).endTime;
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
        Task ret = null;
        for ( int i=0; i < this.schedule.size(); i++){
            if ( this.schedule.get(i).taskID == taskID ){
                ret = this.schedule.get(i);
                this.schedule.remove(ret);
                break;
            }
        }

        return ret;
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


    private boolean isTaskOverlapping(Task t1, Task t2){ 
        return (t1.endTime > t2.startTime && t1.startTime < t2.endTime);
    }
        
    

    

    // TODO ABORT TASK
    // TODO REPLACE TASK

    public int getSize() {
        return this.schedule.size();
    }


    protected void changeTaskOrder() {}

    public void printSchedule(){
        System.out.println("___________________________________SCHEDULE_________________________________________");
        for (Task t : this.schedule){
            System.out.println("time: " + t.startTime + " --> " + t.endTime + "\t taskID: "+t.taskID);
        }

        System.out.println("____________________________________________________________________________________");


    }


    public static void main(String args[]){

        TimeSchedule ts = new TimeSchedule();


        //overlap test
        boolean testOverlap = false;
        boolean testAddFunc = false;
        boolean testSmallFuncs = false;
        boolean testUpdate = true;

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

        if (testSmallFuncs){
            System.out.println("######### testing small funcs #########\n");
            if (!testAddFunc){
                ts.add(new Task(10.0, 20.0));
                ts.add(new Task(30.0, 40.0));
                ts.add(new Task(40.0, 50.0));
                ts.add(new Task(50.0, 100.0));
            }

            System.out.println("######### taskPossible(double start, double end) #########");
            System.out.println( ts.taskPossible(0.0, 5.0) == true ? "success" : "fail");
            System.out.println( ts.taskPossible(20.0, 30.0) == true ? "success" : "fail");
            System.out.println( ts.taskPossible(45.0, 80.0) == false ? "success" : "fail");

            


        }

        if (testUpdate){
            if (!testAddFunc){
                ts.add(new Task(10.0, 20.0, 1));
                ts.add(new Task(30.0, 40.0, 2));
                ts.add(new Task(40.0, 50.0, 3));
                ts.add(new Task(52.0, 100.0, 4));
            }

            System.out.println("######### update(int taskID, double newEndTime) #########");
            ts.update(1, 25.5);
            ts.printSchedule();

            ts.update(1, 31.0);
            ts.printSchedule();

            HashMap<Integer, Double> ret = ts.update(1, 36.0);
            System.out.println(ret);
            ts.printSchedule();
        }
    }
}
