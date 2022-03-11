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




    public RobotAgent(int r_id, TrajectoryEnvelopeCoordinatorSimulation tec, ReedsSheppCarPlanner mp){       //constructor
        System.out.println("################################## constructor ##################################################");
        
        this.robotID = r_id;
        this.tec = tec;
        this.mp = mp;
    }

    public RobotAgent(int r_id, TrajectoryEnvelopeCoordinatorSimulation tec){       //constructor
        System.out.println("################################## constructor ##################################################");
        
        this.robotID = r_id;
        this.tec = tec;
    }


    public void addRobotToSimulation(){
        // add robot to simulation prep...
        double MAX_ACCEL = 10.0;
	    double MAX_VEL = 20.0;

        double xl = 5.0;
	    double yl = 3.7;
        Coordinate f1 = new Coordinate(-xl,yl);
        Coordinate f2 = new Coordinate(xl,yl);
        Coordinate f3 = new Coordinate(xl,-yl);
        Coordinate f4 = new Coordinate(-xl,-yl);

        this.tec.setForwardModel(this.robotID, new ConstantAccelerationForwardModel(
            MAX_ACCEL, 
            MAX_VEL, 
            this.tec.getTemporalResolution(), 
            this.tec.getControlPeriod(), 
            this.tec.getRobotTrackingPeriodInMillis(this.robotID)));

        tec.setFootprint(this.robotID, f1,f2,f3,f4);

        Pose startPos = new Pose(50.0,20.0, Math.PI/2);
        tec.placeRobot(this.robotID, startPos);


        //TODO remove hard coded mission alloc
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

    }


    void communicateState(){
        // talk with agents to form a task


        /*
        Mission m = ...

        this.plan(m);

        */
    }

    void planState(){

        // execute mission

        /*
        for action in plan:
            do action

            if(action not possible):
                replan..
            
        this.communcicate();


        */
    }

    void replanState(){
        // try replan

        /*
        if replan !possible:


        */

    }

    void listener(){
        // listen to incoming msgs
    }

}