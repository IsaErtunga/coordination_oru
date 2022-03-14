package se.oru.coordination.coordination_oru.MAS;
import java.util.ArrayList;

/*
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;

import org.sat4j.ExitCode;


import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
*/
import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.MAS.MessagingSystem;


// public class RobotAgent {
//     protected int robotID;
//     //protected ArrayList<Mission> missions = new ArrayList<Mission>();
//     protected TrajectoryEnvelopeCoordinatorSimulation tec;
//     protected ReedsSheppCarPlanner motionPlanner;


//     public RobotAgent(int a){       //constructor
//         this.robotID = a;
//         System.out.println("####################################################################################");
//         System.out.println(this.robotID);
//         System.out.println("####################################################################################");
//     }


//     public void taskAllocator(ArrayList<Mission> missions){

//         Pose startPoseRobot1 = new Pose(50.0,20.0, Math.PI/2);	
//         Pose startPoseRobot2 = new Pose(50.0,190.0, 3*Math.PI/2);

//         // missions for R1
//         this.motionPlanner.setStart(startPoseRobot1);
//         this.motionPlanner.setGoals(startPoseRobot2);
//         if (!this.motionPlanner.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + startPoseRobot2);
//         PoseSteering[] p1 = this.motionPlanner.getPath();
//         PoseSteering[] p2 = this.motionPlanner.getPathInv();

//         Thread t = new Thread() {
//             @Override
//             public void run() {
//                 while (true) {
//                     synchronized(missions) {
//                         missions.add(new Mission(1, p1));
//                         missions.add(new Mission(1, p2));
//                     }
    
//                     try { Thread.sleep(8000); }
//                     catch (InterruptedException e) { e.printStackTrace(); }
//                 }
//             }

//         };
//         t.start();
//     }


//     public void setMotionPlanner(){     //TODO write this function
//         this.motionPlanner = new ReedsSheppCarPlanner();
//         this.motionPlanner.setFootprint(f1, f2, f3, f4);
//         this.motionPlanner.setTurningRadius(4.0); 				//default is 1.0
//         //rsp.setDistanceBetweenPathPoints(0.5); 	default is 0.5 
//         this.motionPlanner.setMap(yamlFile);

//     }

//     public void taskExecutor(ArrayList<Mission> missions){

//         while(true){
//             if( missions.size()>0){
//                 Mission m = missions.get(0);

//                 synchronized(this.tec) {
//                     if (this.tec.addMissions(m)) { missions.remove(0); }
//                 }
//             }

//             try { Thread.sleep(2000); }
//                 catch (InterruptedException e) { e.printStackTrace(); }
            
//         }
//     }

//     public void initRobot(){

//         /*          add movement model of robot

//         tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(
//             MAX_ACCEL, 
//             MAX_VEL, 
//             tec.getTemporalResolution(), 
//             tec.getControlPeriod(), 
//             tec.getRobotTrackingPeriodInMillis(robotID)));      */

        
//         /*          set robot geometry

//         double xl = 5.0;
//         double yl = 3.7;

//         Coordinate f1 = new Coordinate(-xl,yl);
//         Coordinate f2 = new Coordinate(xl,yl);
//         Coordinate f3 = new Coordinate(xl,-yl);
//         Coordinate f4 = new Coordinate(-xl,-yl);

//         tec.setFootprint(int robotID, f1,f2,f3,f4);     */


//         // set motion planner
//         this.setMotionPlanner();


//         /*          set robot acceleration and velocity

//         tec.setRobotMaxVelocity(int robotID, double maxVelocity)
//         tec.setRobotMaxAcceleration(int robotID, double maxAcceleration)    */

        
//         /*          place robot

//         Pose startPos = new Pose(double x, double y, double dir);
//         tec.placeRobot(robotID, startPos);     */

//         ArrayList<Mission> missions = new ArrayList<Mission>();

//         this.taskAllocator(missions);

//         this.taskExecutor(missions);

//     }

//     // find position tec.getRobotReport(int robotID).getPose() -> returns current pose of type Pose
    
// }


public class RobotAgent {
    protected int robotID;
    protected TrajectoryEnvelopeCoordinatorSimulation tec;
    protected ReedsSheppCarPlanner mp;
    protected MessagingSystem ms;
    protected Coordinate[] rShape;
    protected Pose startPose;

    public ArrayList<Message> inbox = new ArrayList<Message>();
    public ArrayList<Message> outbox = new ArrayList<Message>();

    protected boolean hasMission;
    protected boolean hasNextMission;


    public RobotAgent(int id){this.robotID = id;}

    public RobotAgent(
        int r_id,
        TrajectoryEnvelopeCoordinatorSimulation tec,
        ReedsSheppCarPlanner mp,
        MessagingSystem ms,
        Pose startPos ){
        
        this.robotID = r_id;
        this.tec = tec;
        this.mp = mp;
        this.ms = ms;
        this.startPose = startPos;

        double xl = 5.0;
	    double yl = 3.7;
        this.rShape = new Coordinate[] {new Coordinate(-xl,yl),new Coordinate(xl,yl),new Coordinate(xl,-yl),new Coordinate(-xl,-yl)};
    }

    public RobotAgent( //constructor
        int r_id,
        TrajectoryEnvelopeCoordinatorSimulation tec,
        ReedsSheppCarPlanner mp,
        MessagingSystem ms,
        Coordinate[] shape ){
        
        this.robotID = r_id;
        this.tec = tec;
        this.mp = mp;
        this.ms = ms;
        this.rShape = shape;
    }


    public void addRobotToSimulation(){
        // add robot to simulation prep...
        double MAX_ACCEL = 10.0;
	    double MAX_VEL = 20.0;

        this.tec.setForwardModel(this.robotID, new ConstantAccelerationForwardModel(
            MAX_ACCEL, 
            MAX_VEL, 
            this.tec.getTemporalResolution(), 
            this.tec.getControlPeriod(), 
            this.tec.getRobotTrackingPeriodInMillis(this.robotID)));

        tec.setFootprint(this.robotID, this.rShape);

        tec.placeRobot(this.robotID, this.startPose);

        // join messageSystem
        //this.ms.enterMessageSystem(this.robotID);

        //this.ms.putMessage(this.robotID, "aklfdasfdsafsa");

        //this.ms.getMessage(this.robotID);


        //TODO remove hard coded mission alloc
        /*
        Pose startPoseRobot1 = new Pose(50.0,20.0, Math.PI/2);	
        Pose startPoseRobot2 = new Pose(50.0,190.0, 3*Math.PI/2);

        // missions for R1
        this.mp.setStart(startPoseRobot1);
        this.mp.setGoals(startPoseRobot2);
        if (!this.mp.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + startPoseRobot2);
        PoseSteering[] p1 = this.mp.getPath();


        this.tec.addMissions(new Mission(this.robotID, p1));

        while(true){
            //asd
        }
        */

    }


    public void communicateState(int i){
        // talk with agents to form a task


        /*
        if (!hasNextMission):
            plan next mission...
            
            Mission m = ...       

            this.planState(m);

        */
        Pose goal = new Pose(0,0,0);

        while (this.tec.getRobotReport(this.robotID).getCriticalPoint() != -1){
            try { Thread.sleep(1000); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }

        if (i == 1) goal = new Pose(10.0, 15.0, Math.PI);

        if (i == 2) goal = new Pose(50.0,190.0, 3*Math.PI/2);

        this.planState(goal);

    }

    public void planState(Pose goal){

        // execute mission

        /*
        for action in plan:
            do action

            if(action not possible):
                replan..
            
        this.communcicate();
        */
        //tec.getRobotReport(this.robotID).getPose();

        this.mp.setStart(tec.getRobotReport(this.robotID).getPose());
        this.mp.setGoals(goal);
        if (!this.mp.plan()) throw new Error ("No path between " + "current_pos" + " and " + goal);
        PoseSteering[] p1 = this.mp.getPath();
        

        this.tec.addMissions(new Mission(this.robotID, p1));

        this.communicateState(2);

    }

    void replanState(){
        // try replan

        /*
        if replan !possible:


        */

    }

    void listener(){
        // listen to incoming msgs
        /*
        while (true){


        }


        */
    }

}