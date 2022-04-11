/**
 * @author Alexander Benteby & Isa Ertunga
 * Schedule imbedded in robot agents for coordinating task execution. 
 */
package se.oru.coordination.coordination_oru.MAS;
import java.util.ArrayList;
import java.util.HashMap;
import org.metacsp.multi.spatioTemporal.paths.Pose;


public class TimeSchedule {

    private static double time_sensitivity = 10.0;
    protected Task currentTask = null;    
    protected Pose startPose;
    private ArrayList<Task> schedule = new ArrayList<Task>();
    private HashMap<Integer, Task> reservedTasks = new HashMap<Integer, Task>();
    private HashMap<Double, Double> oreState = new HashMap<Double, Double>();


    public TimeSchedule() {}
    public TimeSchedule(Pose lastSetPose) {
        this.startPose = lastSetPose;
    }
    public TimeSchedule(Pose lastSetPose, double startOre) {
        this.startPose = lastSetPose;
        this.oreState.put(0.0, startOre);
    }


    public int getSize() {
        return this.schedule.size();
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


    public int getOreStateSize () {
        return this.oreState.size();
    }


    /** getNextStartTime() will retrive the next time available for scheduling. 
     * return type is double
     * @return the endTime of the last Task in schedule. it does NOT care if task is reserved or not.
     */
    public double getNextStartTime(){ //return endTime for last task
        synchronized(this.schedule){
            if ( this.schedule.size() <=0 ){
                if (this.currentTask != null) return this.currentTask.endTime;

                else return -1.0;
            }

            return this.schedule.get(this.schedule.size()-1).endTime;
        }
    } 


    /**
     * Returns the of where the agent completed its last task. 
     * Create a lastSetPose in constructor, use it whenever we have no tasks. 
     * @return
     */
    public Pose getLastToPose(){
        if (this.schedule.size() <= 0){
            if ( this.currentTask == null ) return this.startPose;
            else return this.currentTask.toPose;
        }
        return this.schedule.get(this.schedule.size()-1).toPose;
    }


    public boolean setTaskActive(int taskID){
        Task t = this.reservedTasks.get(taskID);
        this.reservedTasks.remove(taskID);
        t.isActive = true;

        return this.add(t);
    }


    private boolean isTaskOverlapping(Task t1, Task t2){ 
        return (t1.endTime > t2.startTime && t1.startTime < t2.endTime);
    }


    private ArrayList<Task> getActiveTasks(){ //TODO remove, old, not used anymore
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
            Task task = new Task(start, end);

            for (Task t : this.schedule){
                if ( this.isTaskOverlapping(t, task) ) return false;
            }

            return true;
        }
    }
    public boolean taskPossible(Task t){
        return this.taskPossible(t.startTime, t.endTime);
    } 


    /** will evaluate how good a timeslot is in the schedule.
     * 
     * @param start start time of slot
     * @param end end time of slot
     * @return an int where higher is better, and 0 is not possible
     */
    public int evaluateTimeSlot(double start, double end){

        return 0;
    }


    /** WE DONT WANT TO USE THIS! REMOVE WHEN POSSIBLE
     * Checks if last task was put by an SA. 
     * @return
     */
    public boolean lastTaskSA() { 
        synchronized(this.schedule){
            ArrayList<Task> tasks = this.getActiveTasks();
            if (tasks.size() > 0) {
                Boolean lastTaskSA = tasks.get(tasks.size() - 1).isSaTask;
                // check if SA
                if (lastTaskSA) {
                    return true;
                }
                return false;
            }
            return false; 
        }
    } 


    /**
     * Returns the amount of ore after the endTime of the last task. 
     * @return
     */
    public double checkEndStateOreLvl(){    // return the last state
        double endTime = -1.0;
        double ore = 0.0;

        for (Double key : this.oreState.keySet()) {
            if ( endTime < key ){
                endTime = key;
                ore = this.oreState.get(key);
            } 
        }
        return ore;
    } 



    protected boolean add(Task task) {
        if (!task.isActive){
            this.reservedTasks.put(task.taskID, task);
            return true;
        }

        synchronized(this.schedule){
            if (this.schedule.size() <= 0){             // case size = 0
                this.schedule.add(task);
                this.addOreState(task);
                return true;
            }

            int scSize = this.schedule.size();

            //System.out.println("t <"+task.startTime +", "+task.endTime+">");

            for (int i=0; i<scSize; i++){               // case size > 0
                Task curr = this.schedule.get(i);

                //System.out.println("\tcurr <"+curr.startTime +", "+curr.endTime+">");

                if (this.isTaskOverlapping(task, curr)) return false;   // task does not fit in schedule

                if ( curr.startTime < task.startTime && i != scSize-1) continue;

                //System.out.println("\tcurr.startTime < task.startTime && i != scSize-1 -->"+(curr.startTime < task.startTime && i != scSize-1));

                i = curr.startTime < task.startTime ? i+1 : i;  // add task after curr if true

                //System.out.println("\ti -->"+i);

                this.schedule.add(i, task); // add task to schedule
                this.addOreState(task);
                break;
            }
            return true;
        }
    }


    private void addOreState(Task t){
        if ( this.oreState.size() > 0 ){
            this.oreState.put(t.endTime, (this.checkEndStateOreLvl() + t.ore)); // prev state + oreChange
            return;
        }
        this.oreState.put(t.endTime, t.ore);    // no prev state = just add
    }


    public Task remove(int taskID) {
        synchronized(this.schedule){

            Task ret = null; //TODO does this work?

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
            Task ret = null;

            if ( this.schedule.size() <=0 ) return null;

            ret = this.schedule.get(0);
            this.currentTask = ret;
            this.schedule.remove(ret);
            return ret;

        }
    }
    

    public void printSchedule(String c){
        String e = "\033[0m";
        
        System.out.println(c+"_____________________________SCHEDULE__________________"+e);
        for (Task t : this.schedule){
            System.out.println(c+"---------------------------------------------------"+e);
            System.out.println(c+"time: " + t.startTime + " --> " + t.endTime + "\t taskID: "+t.taskID + "\twith: "+ t.partner+e);
            System.out.println(c+"from: " +t.fromPose.toString() + " --> " + t.toPose.toString()+e);
        }

        System.out.println(c+"____________________________RESERVED___________________"+e);
        for (int key : this.reservedTasks.keySet()) {
            Task t = this.reservedTasks.get(key);
        
            System.out.println(c+"---------------------------------------------------"+e);
            System.out.println(c+"time: " + t.startTime + " --> " + t.endTime + "\t taskID: "+t.taskID + "\twith: "+ t.partner+e);
            System.out.println(c+"from: " +t.fromPose.toString() + " --> " + t.toPose.toString()+e);
        }
        System.out.println(c+"ORESTATE: " + this.oreState+e);
        

    }












    public static void main(String args[]){

        TimeSchedule ts = new TimeSchedule();


        //overlap test
        boolean testOverlap = false;
        boolean testAddFunc = true;
        boolean testSmallFuncs = false;
        boolean testUpdate = false;
        boolean testOreState = false;
        boolean testSetActive = false;
        boolean testLastToPose = false;

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

            //add first to 0
            System.out.println( ts.add(new Task(10.0, 20.0, 1)) == true ? "add first success" : "add first fail");

            //add to schedule w 1
            System.out.println( ts.add(new Task(5.0, 10.0, 2)) == true ? "success" : "fail");
            ts.remove(2);
            System.out.println( ts.add(new Task(20.0, 25.0, 2)) == true ? "success" : "fail");
            ts.remove(2);

            System.out.println( ts.add(new Task(9.0, 21.0, 2)) == false ? "success" : "fail");
            ts.remove(2);
            System.out.println( ts.add(new Task(11.0, 19.0, 2)) == false ? "success" : "fail");
            ts.remove(2);

            System.out.println( ts.add(new Task(5.0, 11.0, 2)) == false ? "success" : "fail");
            ts.remove(2);
            System.out.println( ts.add(new Task(19.0, 25.0, 2)) == false ? "success" : "fail");

        }

        if (testSmallFuncs){
            System.out.println("######### testing small funcs #########\n");
            if (!testAddFunc){
                ts.add(new Task(10.0, 20.0, 7));
                ts.add(new Task(30.0, 40.0, 8));
                ts.add(new Task(40.0, 50.0, 9));
                ts.add(new Task(50.0, 100.0, 10));
            }

            System.out.println("######### taskPossible(double start, double end) #########");
            System.out.println( ts.taskPossible(0.0, 5.0) == true ? "success" : "fail");
            System.out.println( ts.taskPossible(20.0, 30.0) == false ? "success" : "fail");
            System.out.println( ts.taskPossible(45.0, 80.0) == false ? "success" : "fail");

            


        }

        if (testUpdate){
            if (!testAddFunc){
                ts.add(new Task(10.0, 20.0, 11));
                ts.add(new Task(30.0, 40.0, 12));
                ts.add(new Task(40.0, 50.0, 13));
                ts.add(new Task(52.0, 100.0, 14));
            }

            System.out.println("######### update(int taskID, double newEndTime) #########");
            // ts.update(1, 25.5);
            // ts.printSchedule();

            // ts.update(1, 31.0);
            // ts.printSchedule();

            ts.printSchedule("");
            ArrayList<Task> ret = ts.update(1, 36.0);
            System.out.println("res "+ret.size());
            for(Task t : ret){
                System.out.println("start-->"+t.startTime+"\t end-->"+t.endTime+"\t task-->"+t.taskID);
            }
            
            ts.printSchedule("");
        }

        if (testOreState){
            Task over1 = new Task(5.0, 8.0);
            over1.ore = 15.0;
            Task over2 = new Task(15.0, 20.0);
            over2.ore = -15.0;

            ts.addOreState(over1);
            System.out.println(ts.checkEndStateOreLvl());
            System.out.println(ts.oreState);
            ts.addOreState(over2);
            System.out.println(ts.checkEndStateOreLvl());
            System.out.println(ts.oreState);

        }

        if ( testSetActive ){
            if (!testAddFunc){
                ts.add(new Task(10.0, 20.0, 11));
                ts.add(new Task(30.0, 40.0, 12));
                ts.add(new Task(40.0, 50.0, 13));
                ts.add(new Task(52.0, 100.0, 14));
            }

            Task t1 = new Task(20.0, 30.0, 1);
            t1.isActive = false;
            Task t2 = new Task(30.0, 40.0, 2);
            t2.isActive = false;
            Task t3 = new Task(50.0, 53.0, 3);
            t3.isActive = false;

            System.out.println( ts.add(t1) == true ? "t1 success" : "t1 fail");
            ts.printSchedule("");
            System.out.println( ts.add(t2) == true ? "t2 success" : "t2 fail");
            ts.printSchedule("");
            System.out.println( ts.add(t3) == true ? "t3 success" : "t3 fail");
            ts.printSchedule("");


            System.out.println( ts.setTaskActive(t1.taskID) == true ? "t1 success" : "t1 fail");

            ts.printSchedule("");
        }

        if ( testLastToPose ){
            TimeSchedule tss = new TimeSchedule(new Pose(0.0, 0.0, 0.0));

            Task t1 = new Task(10.0, 20.0, 1);
            Task t2 = new Task(30.0, 40.0, 2);
            Task t3 = new Task(50.0, 60.0, 3);

            t1.fromPose = new Pose(0.0, 0.0, 0.0);
            t1.toPose = new Pose(0.0, 2.0, 0.0);
            
            t2.fromPose = new Pose(0.0, 2.0, 0.0);
            t2.toPose = new Pose(2.0, 2.0, 0.0);

            t3.fromPose = new Pose(2.0, 2.0, 0.0);
            t3.toPose = new Pose(2.0, 10.0, 0.0);

            System.out.println(tss.getLastToPose().toString());

            tss.add(t1);
            //tss.printSchedule();
            System.out.println(tss.getLastToPose().toString());

            tss.add(t2);
            //tss.printSchedule();

            System.out.println(tss.getLastToPose().toString());

            tss.add(t3);
            //tss.printSchedule();

            System.out.println(tss.getLastToPose().toString());

        }
        
    }
}
