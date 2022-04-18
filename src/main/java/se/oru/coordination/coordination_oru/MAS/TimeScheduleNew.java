package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class TimeScheduleNew {
    // control parameters
    private static double tSensitivity = 0.0;
    private static int unchangeableTaskCount = 0;

    private double capacity; 
    protected Task currentTask;    
    private ArrayList<Task> schedule = new ArrayList<Task>();
    private HashMap<Integer, Task> reserved = new HashMap<Integer, Task>();
    public OreState oreState = null;

    public TimeScheduleNew(Pose startPos, double oreCapacity, double startOre){
        this.oreState = new OreState(oreCapacity, startOre);
        this.capacity = oreCapacity;
        Task initialTask = new Task(-1.0, -1.0);
        initialTask.toPose = startPos;
        this.currentTask = initialTask;
    }
    public TimeScheduleNew(OreState os, Pose startPos, double oreCapacity, double startOre){
        this.oreState = os;
        Task initialTask = new Task(-1.0, -1.0);
        initialTask.toPose = startPos;
        this.currentTask = initialTask;
    }

    private Task getLastTask(){
        if ( this.schedule.size() <= 0 ) return this.currentTask;
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

        boolean taskAdded = this.addEvent(t);
        double lastOreState;
        synchronized(this.oreState){ lastOreState = this.oreState.getLastOreState(); }

        if ( taskAdded && (lastOreState <= -0.1 || lastOreState > this.capacity+0.1) ){
            this.abortEvent(taskID);
            synchronized(this.oreState){ this.oreState.removeState(taskID); }
            return false;
        }
        return true;
    }

    /**
     * Overloaded for SA agent
     * @param taskID
     * @param isSA
     * @return
     */
    public boolean setEventActive(int taskID, boolean isSA){
        Task t = this.reserved.remove(taskID);
        if( t == null ) return false;
        t.isActive = true;

        boolean taskAdded = this.addEvent(t);
        double lastOreState;
        synchronized(this.oreState){
            lastOreState = this.oreState.getLastOreState();
        }
        return true;
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
        synchronized(this.oreState){ this.oreState.addState( this.oreState.createState(task.taskID, task.endTime, task.ore) ); }

        return true;
    }

    public Task getEvent(int taskID){
        for (Task t : this.schedule ){
            if ( t.taskID == taskID ) return t;
        }
        return this.reserved.get(taskID); 
    }

    public Pose getPoseAtTime(double time){
        if ( this.schedule.size() <= 0 || (int)(time) == -1) return this.currentTask.toPose;
            
        for ( int i=this.schedule.size()-1; i>=0; i-- ){
            Task curr = this.schedule.get(i);
            if ( curr.endTime <= time ) return curr.toPose;
        }
        return null;
    }
    public Pose getNextPose(){
        return this.getPoseAtTime(Double.MAX_VALUE);
    }

     /**
      * will push all slots in schedule so they are compressed with no free space between.
      * @param nextStartTime is the calculated next starttime for the next task.
      * @return an array of tasks to be informed of the new times.
      */
    public ArrayList<Task> compressSchedule(double nextStartTime){
        ArrayList<Task> tasks2inform = new ArrayList<Task>();
        final int startIndex = this.unchangeableTaskCount;
        int scheduleSize = this.schedule.size();

        if ( scheduleSize > 0 ){
            Task prev = this.schedule.get(0);
            double taskLength = prev.endTime - prev.startTime;
            prev.startTime = nextStartTime;
            prev.endTime = nextStartTime + taskLength;
            synchronized(this.oreState){ this.oreState.changeState(prev.taskID, prev.endTime); }
            if ( startIndex <= 0 ) tasks2inform.add(prev);
            for ( int i=1; i<scheduleSize; i++ ){
                Task curr = this.schedule.get(i);
                double newEndTime = curr.endTime - curr.startTime + prev.endTime;

                synchronized(this.oreState){ this.oreState.changeState(curr.taskID, newEndTime); }
                curr.endTime = newEndTime;
                curr.startTime = prev.endTime;
                if ( startIndex <= i ) tasks2inform.add(curr);
                
                prev = curr;
            }
        }
        return tasks2inform;
    }

    /**
     * will check if new times of task can be updated. if it doesnt work with task after, the task after will be aborted.
     * if it doesnt work with task before, it will abort the task to be updated. always prio tasks closer in future.
     * @param taskID id of task to be updated
     * @param NewEndTime new endTime of task to be updated
     * @return the robotID to inform of their task being aborted. if -1 then abort the task being altered.
     */
    public Task updateTaskEndTimeIfPossible(int taskID, double NewEndTime){
        Task taskToAbort = null;
        Task t = this.getEvent(taskID);
        if ( t == null ) return null;

        double newStartTime = NewEndTime - (t.endTime - t.startTime);
        int indexOfTask = this.schedule.indexOf(t);
        if ( indexOfTask < 0 ) return null;

        boolean earlierStartTime = t.endTime > NewEndTime;
        t.startTime = newStartTime;
        t.endTime = NewEndTime;

        if ( earlierStartTime && indexOfTask > 0){ // if task is earlier than expected
            Task taskBefore = this.schedule.get(indexOfTask-1);
            if ( this.tasksOverlapping(newStartTime, NewEndTime, taskBefore.startTime, taskBefore.endTime) == true ){ // if overlapp with task before
                taskToAbort = taskBefore.startTime > t.startTime ? taskBefore : t;
            }
        }
        else if ( this.schedule.size()-1 > indexOfTask ){   // else task is later. if taskID has task after it, then check if overlapp.
            Task taskAfter = this.schedule.get(indexOfTask+1);
            if ( this.tasksOverlapping(newStartTime, NewEndTime, taskAfter.startTime, taskAfter.endTime) == true ){
                taskToAbort = taskAfter.startTime > t.startTime ? taskAfter : t;
            }
        }
        if ( taskToAbort != null ) this.abortEvent(taskToAbort.taskID);
        synchronized(this.oreState){ this.oreState.changeState(taskID, NewEndTime); }
        return taskToAbort;
    }

    /*
    private Task getTaskAtTime(double sTime, double eTime){
        ArrayList<Task> tasksAtTime = new ArrayList<Task>();
        for ( Task t : this.schedule ){
            if ( t.startTime >= time && t.endTime <= time ) tasksAtTime.add(t);
        }
        return tasksAtTime;
    }
    */


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
        synchronized(this.oreState){ this.oreState.removeState(taskID); }
        return this.removeEvent(taskID);
    }

    public Task getNextEvent(){
        if( this.schedule.size() <= 0) return null;
        Task t = this.schedule.remove(0);
        this.currentTask = t;
        return t;
    }

    public ArrayList<Task> fixBrokenSchedule(){
        ArrayList<Task> fixes = new ArrayList<Task>();
        int taskIDwhereBroken = -1;

        synchronized(this.oreState){ taskIDwhereBroken = this.oreState.getFirstFail(); }
        if ( taskIDwhereBroken == -1 ) return fixes;
        
        ArrayList<Task> scheduleCopy = new ArrayList<Task>(this.schedule);
        
        int indexOfBrokenTask = this.schedule.indexOf(this.getEvent(taskIDwhereBroken));
        if ( indexOfBrokenTask >= scheduleCopy.size() || indexOfBrokenTask < 0 ) return fixes;

        System.out.println("\t\tIn fixBrokenSchedule, taskID where schedule broken-->"+taskIDwhereBroken+"\tindex of that task-->"+indexOfBrokenTask+"\n");
        //this.printSchedule("");
        for (int i=indexOfBrokenTask; i < scheduleCopy.size(); i++){
            Task t = scheduleCopy.get(i);
            this.abortEvent(t.taskID);
            fixes.add(t);
        }
        return fixes;
    }

    //-----------oreState retrives from timeSchedule-----------
    public double getLastOreState(){
        synchronized(this.oreState){return this.oreState.getLastOreState();}
    }
    public double getOreStateAtTime(double time){
        synchronized(this.oreState){return this.oreState.getStateAtTime(time);}
    }
    public ArrayList<Integer> getOreStateInconsistencies(){
        synchronized(this.oreState){return this.oreState.getInconsistencies();}
    }

    public boolean changeOreStateEndTime(int taskID, double newEndTIme){
        synchronized(this.oreState){return this.oreState.changeState(taskID, newEndTIme); }        
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
        System.out.println(c+"_____________________________ORE-STATE__________________"+e);
        synchronized(this.oreState){ this.oreState.print(c); }
        
    }

    private boolean tasksOverlapping( double t1StartTime, double t1EndTime, double t2StartTime, double t2EndTime){
        return (t1EndTime > t2StartTime && t1StartTime < t2EndTime);
    }
    private boolean tasksOverlapping( Task t1, double tStartTime, double tEndTime){
        return this.tasksOverlapping(t1.startTime,t1.endTime, tStartTime, tEndTime);
    }
    private boolean tasksOverlapping( Task t1, Task t2){
        return this.tasksOverlapping(t1, t2.startTime, t2.endTime);
    }
    

    public boolean isTaskPossible(int taskID, double tStartTime, double tEndTime){
        for ( Task t : this.schedule ){
            if (t.taskID == taskID) continue;
            if (this.tasksOverlapping(t, tStartTime, tEndTime) ) return false;
        }
        return true;
    }
    public boolean isTaskPossible(Task t){
        return this.isTaskPossible(t.taskID, t.startTime, t.endTime);
    }


    public int evaluateEventSlot(){

        return 0;
    }

    public static void main(String[] args){
        Pose startp = new Pose(0.0, 0.0, 0.0);
        double startOre = 0.0;
        double capacity = 30.0;
        TimeScheduleNew ts = new TimeScheduleNew(startp, capacity, capacity);


        //overlap test
        boolean testOverlap = false;
        boolean testAddFunc = false;
        boolean testSmallFuncs = false;
        boolean testUpdate = true;
        boolean testSetActive = false;
        boolean testLastToPose = false;

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
            System.out.println( ts.isTaskPossible(999, 0.0, 5.0) == true ? "success" : "fail");
            System.out.println( ts.isTaskPossible(999, 20.0, 30.0) == false ? "success" : "fail");
            System.out.println( ts.isTaskPossible(999, 45.0, 80.0) == false ? "success" : "fail");
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
                t3.ore = 15.0;
                ts.addEvent(t0);
                ts.addEvent(t1);
                ts.addEvent(t2);
                ts.addEvent(t3);
            }

            System.out.println("######### update(int taskID, double newEndTime) #########");
            System.out.println("\nBEFORE");
            ts.printSchedule("");


            System.out.println(ts.getOreStateInconsistencies());
                    
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
