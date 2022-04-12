package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class TimeScheduleNew {
    // control parameters
    private static double tSensitivity = 10.0;
    private static int unchangeableTaskCount = 2;


    protected Task currentTask = null;    
    private ArrayList<Task> schedule = new ArrayList<Task>();
    private HashMap<Integer, Task> reserved = new HashMap<Integer, Task>();
    public OreState oreState = null;

    public TimeScheduleNew(Pose startPos, double oreCapacity, double startOre){
        this.oreState = new OreState(oreCapacity, startOre);
    }
    public TimeScheduleNew(OreState os, Pose startPos, double oreCapacity, double startOre){
        this.oreState = os;
    }

    public boolean setEventActive(int taskID){

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
        this.oreState.addState(this.oreState.createState(task.endTime, task.ore));

        return true;
    }

    public Task getEvent(int taskID){
        return null;
    }

    public Pose getPose(){

        return null;
    }

    public ArrayList<Task> updateSchedule(){
        ArrayList<Task> fixes = new ArrayList<Task>();  // add {Task t} if task updated with diff> 5s

        if (this.schedule.size() > this.unchangeableTaskCount){

            Task prev = this.schedule.get(this.unchangeableTaskCount);
            for ( int i=this.unchangeableTaskCount+1; i<this.schedule.size(); i++ ){
                Task curr = this.schedule.get(i);
                if (prev.endTime - this.tSensitivity > curr.startTime ){
                    curr.endTime = curr.endTime - curr.startTime + prev.endTime;
                    curr.startTime = prev.endTime;
                    fixes.add(curr);
                }
            }

        }
        
        return null;
    }

    public void changeTaskEndTime(int taskID, double newEndTime){
        Task t = this.getEvent(taskID);

        
    }

    public boolean removeEvent(int taskID){
        for (Task t : this.schedule){
            if ( t.taskID == taskID ){
                this.schedule.remove(t);
                this.updateSchedule();
                return true;
            }
        }
        return true;
    }
    public boolean removeEvent(Task t){
        return true;
    }

    public Task fetchNextTask(){
        if( this.schedule.size() <= 0) return null;
        Task t = this.schedule.remove(0);
        this.currentTask = t;
        return t;
    }

    public void printSchedule(){

    }

    private boolean tasksOverlapping( Task t1, Task t2){
        return (t1.endTime > t2.startTime && t1.startTime < t2.endTime);
    }

    public int evaluateEventSlot(){

        return 0;
    }

    public static void main(String[] args){

    }
}
