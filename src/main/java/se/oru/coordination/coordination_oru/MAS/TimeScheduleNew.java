package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class TimeScheduleNew {
    // control parameters
    private static double tSensitivity = 5.0;
    private static int unchangeableTaskCount = 0;

    private Pose startPos = null; 
    protected Task currentTask = null;    
    private ArrayList<Task> schedule = new ArrayList<Task>();
    private HashMap<Integer, Task> reserved = new HashMap<Integer, Task>();
    public OreState oreState = null;

    public TimeScheduleNew(Pose startPos, double oreCapacity, double startOre){
        this.oreState = new OreState(oreCapacity, startOre);
        this.startPos = startPos;
    }
    public TimeScheduleNew(OreState os, Pose startPos, double oreCapacity, double startOre){
        this.oreState = os;
        this.startPos = startPos;
    }

    private Task getLastTask(){
        if ( this.schedule.size() <= 0 ){
            if ( this.currentTask != null ) return this.currentTask;
            return null;
        }
        return this.schedule.get(this.schedule.size()-1);
    }

    public double getNextStartTime(){
        Task t = this.getLastTask();
        if ( t != null ) return t.endTime;
        return -1.0;
    }

    public int getSize(){
        return this.schedule.size();
    }

    public boolean setEventActive(int taskID){
        Task t = this.reserved.remove(taskID);
        if( t == null ) return false;
        t.isActive = true;

        return this.addEvent(t);
    }

    public boolean addEvent(Task task){
        if (!task.isActive){
            this.reserved.put(task.taskID, task);
            return true;
        }

        int index = 0;
        if (this.schedule.size() > 0){             // case size = 0
    
            for (int i=0; i<this.schedule.size(); i++){               // case size > 0
                Task curr = this.schedule.get(i);

                if (this.tasksOverlapping(task, curr)) return false;   // task does not fit in schedule

                if ( curr.startTime < task.startTime && i != this.schedule.size()-1) continue;

                index = curr.startTime < task.startTime ? i+1 : i;  // add task after curr if true
                break;
            }
        }

        this.schedule.add(index, task);
        synchronized(this.oreState){ this.oreState.addState( this.oreState.createState(task.endTime, task.ore) ); }

        return true;
    }

    public Task getEvent(int taskID){
        for (Task t : this.schedule ){
            if ( t.taskID == taskID ) return t;
        }
        return this.reserved.get(taskID); 
    }

    public Pose getPoseAtTime(double time){
        if ( this.schedule.size() <= 0 ){
            if ( this.currentTask != null ) return this.currentTask.toPose;
            return this.startPos;
        }
        for ( int i=this.schedule.size()-1; i>=0; i-- ){
            Task curr = this.schedule.get(i);
            if ( curr.endTime <= time ) return curr.toPose;
        }
        return null;
    }
    public Pose getNextPose(){
        return this.getPoseAtTime(Double.MAX_VALUE);
    }

    public ArrayList<Task> updateSchedule(){
        ArrayList<Task> fixes = new ArrayList<Task>();  // add {Task t} if task updated with diff> 5s

        if (this.schedule.size() > this.unchangeableTaskCount){

            Task prev = this.schedule.get(this.unchangeableTaskCount);
            for ( int i=this.unchangeableTaskCount+1; i<this.schedule.size(); i++ ){
                Task curr = this.schedule.get(i);

                if (prev.endTime - this.tSensitivity > curr.startTime ){
                    double newEndTime = curr.endTime - curr.startTime + prev.endTime;
                    synchronized(this.oreState){ this.oreState.changeState(curr.endTime, curr.ore, newEndTime); }
                    curr.endTime = newEndTime;
                    curr.startTime = prev.endTime;
                    fixes.add(curr);
                }
                prev = curr;
            }
        }
        return fixes;
    }

    public ArrayList<Task> setNewEndTime(int taskID, double newEndTime){
        Task t = this.getEvent(taskID);
        synchronized(this.oreState){ this.oreState.changeState(t.endTime, t.ore, newEndTime); }
        t.endTime = newEndTime;
        return this.updateSchedule();
    }

    public boolean removeEvent(int taskID){
        for (Task t : this.schedule){
            if ( t.taskID == taskID ){
                this.schedule.remove(t);
                return true;
            }
        }
        if ( this.reserved.remove(taskID) != null ) return true;
        return false;
    }
    public boolean removeEvent(Task t){
        return this.removeEvent(t.taskID);
    }

    public boolean abortEvent(int taskID){
        Task t = this.getEvent(taskID);
        synchronized(this.oreState){ this.oreState.removeState(t.endTime, t.ore); }
        return this.removeEvent(t);
    }

    public Task getNextEvent(){
        if( this.schedule.size() <= 0) return null;
        Task t = this.schedule.remove(0);
        this.currentTask = t;
        return t;
    }

    //-----------oreState retrives from timeSchedule-----------
    public double getLastOreState(){
        synchronized(this.oreState){return this.oreState.getLastOreState();}
    }
    public double getOreStateAtTime(double time){
        synchronized(this.oreState){return this.oreState.getStateAtTime(time);}
    }
    public HashMap<Double, Double> getOreStateInconsistencies(){
        synchronized(this.oreState){return this.oreState.getInconsistencies();}
    }
    //---------------------------------------------------------

    public void printSchedule(String c){
        String e = "\033[0m";
        
        System.out.println(c+"_____________________________SCHEDULE__________________"+e);
        for (Task t : this.schedule){
            System.out.println(c+"---------------------------------------------------"+e);
            System.out.println(c+"time: " +String.format("%.2f",t.startTime)  + " --> " + String.format("%.2f",t.endTime) + "\t taskID: "+t.taskID + "\twith: "+ t.partner+e);
            //System.out.println(c+"from: " +t.fromPose.toString() + " --> " + t.toPose.toString()+e);
        }

        System.out.println(c+"____________________________RESERVED___________________"+e);
        for (int key : this.reserved.keySet()) {
            Task t = this.reserved.get(key);
        
            System.out.println(c+"---------------------------------------------------"+e);
            System.out.println(c+"time: " +String.format("%.2f",t.startTime)  + " --> " + String.format("%.2f",t.endTime) + "\t taskID: "+t.taskID + "\twith: "+ t.partner+e);
            //System.out.println(c+"from: " +t.fromPose.toString() + " --> " + t.toPose.toString()+e);
        }
        System.out.println();
        System.out.println("_____________________________ORE-STATE__________________");
        synchronized(this.oreState){ this.oreState.print(c); }
        
    }

    
    private boolean tasksOverlapping( Task t1, double tStartTime, double tEndTime){
        return (t1.endTime > tStartTime && t1.startTime < tEndTime);
    }
    private boolean tasksOverlapping( Task t1, Task t2){
        return tasksOverlapping(t1, t2.startTime, t2.endTime);
    }

    public boolean isTaskPossible(double tStartTime, double tEndTime){
        for ( Task t : this.schedule ){
            if (this.tasksOverlapping(t, tStartTime, tEndTime) ) return false;
        }
        return true;
    }
    public boolean isTaskPossible(Task t){
        return this.isTaskPossible(t.startTime, t.endTime);
    }


    public int evaluateEventSlot(){

        return 0;
    }

    public static void main(String[] args){
        Pose startp = new Pose(0.0, 0.0, 0.0);
        double startOre = 0.0;
        double capacity = 100.0;
        TimeScheduleNew ts = new TimeScheduleNew(startp, startOre, capacity);


        //overlap test
        boolean testOverlap = false;
        boolean testAddFunc = false;
        boolean testSmallFuncs = false;
        boolean testUpdate = false;
        boolean testSetActive = false;
        boolean testLastToPose = true;

        if (testOverlap){
            System.out.println("######### testing tasksOverlapping(Task t, Task t) #########");

            Task over1 = new Task(10.0, 20.0);
            Task over2 = new Task(5.0, 15.0);
            
            System.out.println( ts.tasksOverlapping(over1, over2) == true ? "success" : "fail");

            over2 = new Task(15.0, 25.0);
            System.out.println( ts.tasksOverlapping(over1, over2) == true ? "success" : "fail");

            over2 = new Task(5.0, 25.0);
            System.out.println( ts.tasksOverlapping(over1, over2) == true ? "success" : "fail");

            over2 = new Task(12.0, 18.0);
            System.out.println( ts.tasksOverlapping(over1, over2) == true ? "success" : "fail");

            over2 = new Task(5.0, 10.0);
            System.out.println( ts.tasksOverlapping(over1, over2) == false ? "success" : "fail");

            over2 = new Task(20.0, 25.0);
            System.out.println( ts.tasksOverlapping(over1, over2) == false ? "success" : "fail");
        }


        if (testAddFunc){
            System.out.println("######### testing add(Task t) #########");

            //add first to 0
            System.out.println( ts.addEvent(new Task(10.0, 20.0, 1)) == true ? "add first success" : "add first fail");

            //add to schedule w 1
            System.out.println( ts.addEvent(new Task(5.0, 10.0, 2)) == true ? "success" : "fail");
            ts.removeEvent(2);
            System.out.println( ts.addEvent(new Task(20.0, 25.0, 2)) == true ? "success" : "fail");
            ts.removeEvent(2);

            System.out.println( ts.addEvent(new Task(9.0, 21.0, 2)) == false ? "success" : "fail");
            ts.removeEvent(2);
            System.out.println( ts.addEvent(new Task(11.0, 19.0, 2)) == false ? "success" : "fail");
            ts.removeEvent(2);

            System.out.println( ts.addEvent(new Task(5.0, 11.0, 2)) == false ? "success" : "fail");
            ts.removeEvent(2);
            System.out.println( ts.addEvent(new Task(19.0, 25.0, 2)) == false ? "success" : "fail");

            System.out.println( ts.addEvent(new Task(20.0, 25.0, 3)) == true ? "success" : "fail");
            System.out.println( ts.addEvent(new Task(30.0, 40.0, 4)) == true ? "success" : "fail");
            System.out.println( ts.addEvent(new Task(25.0, 30.0, 5)) == true ? "success" : "fail");
            ts.printSchedule("");
        }

        if (testSmallFuncs){
            System.out.println("######### testing small funcs #########\n");
            if (!testAddFunc){
                ts.addEvent(new Task(10.0, 20.0, 7));
                ts.addEvent(new Task(30.0, 40.0, 8));
                ts.addEvent(new Task(40.0, 50.0, 9));
                ts.addEvent(new Task(50.0, 100.0, 10));
            }

            System.out.println("######### taskPossible(double start, double end) #########");
            System.out.println( ts.isTaskPossible(0.0, 5.0) == true ? "success" : "fail");
            System.out.println( ts.isTaskPossible(20.0, 30.0) == false ? "success" : "fail");
            System.out.println( ts.isTaskPossible(45.0, 80.0) == false ? "success" : "fail");
        }

        if (testUpdate){
            if (!testAddFunc){
                Task t0 = new Task(10.0, 20.0, 1);
                t0.ore = -15.0;
                Task t1 = new Task(30.0, 40.0, 2);
                t1.ore = -15.0;
                Task t2 = new Task(40.0, 50.0, 3);
                t2.ore = -15.0;
                Task t3 = new Task(52.0, 100.0, 4);
                t3.ore = -15.0;
                ts.addEvent(t0);
                ts.addEvent(t1);
                ts.addEvent(t2);
                ts.addEvent(t3);
            }

            System.out.println("######### update(int taskID, double newEndTime) #########");
            System.out.println("\nBEFORE");
            ts.printSchedule("");

            ts.setNewEndTime(1, 31.0);
            //ts.printSchedule("");
            System.out.println("\nnow with conflict::");
            ts.setNewEndTime(1, 36.0);
            ts.printSchedule("");
            ArrayList<Task> aa = ts.setNewEndTime(2, 55.0);
            ts.printSchedule("");

            for ( Task t : aa ){
                System.out.println("taskid-->"+t.taskID +"\tsTime-->"+t.startTime+"\teTime-->"+t.endTime);
            }
            ts.printSchedule("");
                    
        }

        if ( testSetActive ){
            if (!testAddFunc){
                ts.addEvent(new Task(10.0, 20.0, 11));
                ts.addEvent(new Task(30.0, 40.0, 12));
                ts.addEvent(new Task(40.0, 50.0, 13));
                ts.addEvent(new Task(52.0, 100.0, 14));
            }

            Task t1 = new Task(20.0, 30.0, 1);
            t1.isActive = false;
            Task t2 = new Task(30.0, 40.0, 2);
            t2.isActive = false;
            Task t3 = new Task(50.0, 53.0, 3);
            t3.isActive = false;

            System.out.println( ts.addEvent(t1) == true ? "t1 success" : "t1 fail");
            ts.printSchedule("");
            System.out.println( ts.addEvent(t2) == true ? "t2 success" : "t2 fail");
            ts.printSchedule("");
            System.out.println( ts.addEvent(t3) == true ? "t3 success" : "t3 fail");
            ts.printSchedule("");


            System.out.println( ts.setEventActive(t1.taskID) == true ? "t1 success" : "t1 fail");

            ts.printSchedule("");
        }

        if ( testLastToPose ){
            TimeScheduleNew tss = new TimeScheduleNew(startp, startOre, capacity);

            Task t1 = new Task(10.0, 20.0, 1);
            Task t2 = new Task(30.0, 40.0, 2);
            Task t3 = new Task(50.0, 60.0, 3);

            t1.fromPose = new Pose(0.0, 0.0, 0.0);
            t1.toPose = new Pose(0.0, 2.0, 0.0);
            
            t2.fromPose = new Pose(0.0, 2.0, 0.0);
            t2.toPose = new Pose(2.0, 2.0, 0.0);

            t3.fromPose = new Pose(2.0, 2.0, 0.0);
            t3.toPose = new Pose(2.0, 10.0, 0.0);

            System.out.println(tss.getNextPose().toString());

            tss.addEvent(t1);
            //tss.printSchedule();
            System.out.println(tss.getNextPose().toString());

            tss.addEvent(t2);
            //tss.printSchedule();

            System.out.println(tss.getNextPose().toString());

            tss.addEvent(t3);
            //tss.printSchedule();

            System.out.println(tss.getNextPose().toString());
        }
    }

}
