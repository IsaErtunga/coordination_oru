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


    public Pose getNextPos(){
        return null;
    }

    public boolean setTaskActive(int taskID){

        return true;
    }

    private ArrayList<Task> getActiveTasks(){
        ArrayList<Task> ret = new ArrayList<Task>(this.schedule);
        ret.removeIf(i -> i.isActive == false); 
        return ret;
    }

    /** update() will change the endTime of a task and look if there are conflicts in the schedule after. 
     *  It will alter the schedule fix the delay and return all tasks that have been changed too much.
     *  inform-msgs should be sent to the taskProvider of all tasks returned, informing of the update.
     * @param taskID id of the task to be updated
     * @param newEndTime the new time of task updated
     * @return a list of tasks that NEED to be informed of schedule changes.
     */
    public ArrayList<Task> update(int taskID, double newEndTime){   //TODO DONT TEST! NOT DONE! WORK IN PROGRESS.
        ArrayList<Task> ret = new ArrayList<Task>();  // add {Task t} if task updated with diff> 5s

        synchronized(this.schedule){
            ArrayList<Task> tasks = this.getActiveTasks();
            
            if (tasks.size() ==1){
                Task t = tasks.get(0);
                if ( newEndTime - t.endTime > this.time_sensitivity ){
                    ret.add(t);
                } 
                t.startTime = newEndTime -(t.endTime - t.startTime);
                t.endTime = newEndTime;
                
                return ret;
            } 

            for ( int i=0; i<tasks.size(); i++ ){      
                Task curr = tasks.get(i);

                //System.out.println(" curr <"+curr.startTime+" ,"+curr.endTime+" >");
                //System.out.println(" curr.taskID == taskID -->" + (curr.taskID == taskID));

                if ( curr.taskID == taskID ){   // found the task being updated
                    curr.startTime = newEndTime -(curr.endTime - curr.startTime);
                    curr.endTime = newEndTime;

                    // System.out.println(" i != tasks.size()-1 -->" + (i != tasks.size()-1));
                    // System.out.println(" curr.endTime - tasks.get(i+1).startTime > this.time_sensitivity -->" + (curr.endTime - tasks.get(i+1).startTime > this.time_sensitivity));

                    if ( i != tasks.size()-1 && curr.endTime - tasks.get(i+1).startTime > this.time_sensitivity){
                        ret.add(curr);

                        for ( int j=i+1; j<tasks.size(); j++ ){ // for every task that may be affected by overlap
                            curr = tasks.get(j);

                            // System.out.println("\t curr <"+curr.startTime+" ,"+curr.endTime+" >");
                            // System.out.println("\t tasks.get(j-1).endTime - curr.startTime > this.time_sensitivity -->" + (tasks.get(j-1).endTime - curr.startTime > this.time_sensitivity));

                            if (tasks.get(j-1).endTime - curr.startTime > this.time_sensitivity){ // if schedule 
                                double diff = tasks.get(j-1).endTime - curr.startTime;
                                curr.startTime += diff;
                                curr.endTime += diff;
                                ret.add(curr);
                            }
                        }
                    }

                    break;
                }
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
        synchronized(this.schedule){
            ArrayList<Task> tasks = this.getActiveTasks();

            Task task = new Task(start, end);
            for (int i=0; i<tasks.size(); i++){
                if ( this.isTaskOverlapping(tasks.get(i), task) ) return false;
            } 
                
            return true;
        }
    } 


    /** getNextStartTime() will retrive the next time available for scheduling. 
     * return type is double
     * @return the endTime of the last Task in schedule. it does NOT care if task is reserved or not.
     */
    public double getNextStartTime(){ //return endTime for last task
        synchronized(this.schedule){
            ArrayList<Task> tasks = this.getActiveTasks();
            if( tasks.size() > 0 ) return tasks.get(tasks.size()-1).endTime;
            else return -1.0;
        }
    } 


    public double checkEndStateOreLvl(){
        return 0.0;
    } // return the last state


    protected boolean add(Task task) {
        synchronized(this.schedule){
            //task.isActive;
            if (!task.isActive){
                for (int i=0; i<this.schedule.size(); i++){                     // case size > 0
                    Task curr = this.schedule.get(i);
                    if ( curr.startTime >= task.startTime ){
                        this.schedule.add(i, task);
                    }
                }
            }

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
    }


    public Task remove(int taskID) {
        synchronized(this.schedule){

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
    }

    /**
     * 
     * @return
     */
    public Task pop() {
        synchronized(this.schedule) {
            ArrayList<Task> tasks = this.getActiveTasks();
            Task ret = null;
            if (tasks.size() > 0) {
                ret = tasks.get(0);
                this.schedule.remove(ret);
            }
            return ret;
        }
    }
    
    /**
     * Returns the of where the agent completed its last task. 
     * @return
     */
    public Pose getLastToPose() {
        return new Pose(50.0,190.0, 3*Math.PI/2);
    }

    public Task get(int taskID) {
        synchronized(this.schedule){

            for (Task task : this.schedule) {
                if (task.taskID == taskID) {
                    return task;
                }
            }
            return null;
        }
    }


    private boolean isTaskOverlapping(Task t1, Task t2){ 
        return (t1.endTime > t2.startTime && t1.startTime < t2.endTime);
    }


    public int getSize() {
        synchronized(this.schedule){
            return this.schedule.size();
        }
    }


    public void printSchedule(){
        System.out.println("___________________________________SCHEDULE_________________________________________");
        for (Task t : this.schedule){
            System.out.println("time: " + t.startTime + " --> " + t.endTime + "\t taskID: "+t.taskID);
        }

        System.out.println("____________________________________________________________________________________");

    }


    public static void main(String args[]){

        ArrayList<Task> t1 = new ArrayList<Task>();



        /*

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
            // ts.update(1, 25.5);
            // ts.printSchedule();

            // ts.update(1, 31.0);
            // ts.printSchedule();

            ts.printSchedule();
            ArrayList<Task> ret = ts.update(1, 36.0);
            System.out.println("res "+ret.size());
            for(Task t : ret){
                System.out.println("start-->"+t.startTime+"\t end-->"+t.endTime+"\t task-->"+t.taskID);
            }
            
            ts.printSchedule();
        }
        */
    }
}
