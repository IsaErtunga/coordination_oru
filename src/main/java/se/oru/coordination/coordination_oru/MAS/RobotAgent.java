package se.oru.coordination.coordination_oru.MAS;

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


public class RobotAgent {
    protected int robotID;
    protected TrajectoryEnvelopeCoordinatorSimulation tec;
    protected ReedsSheppCarPlanner mp;
    protected Coordinate[] rShape;
    protected Pose startPose;

    public ArrayList<Message> inbox = new ArrayList<Message>();
    public ArrayList<Message> outbox = new ArrayList<Message>();


    public RobotAgent(int id){this.robotID = id;}   // for testing

    public RobotAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
                        ReedsSheppCarPlanner mp, Pose startPos, Router router ){
        
        this.robotID = r_id;
        this.tec = tec;
        this.mp = mp;
        this.startPose = startPos;

        router.enterNetwork(this);

        double xl = 5.0;
	    double yl = 3.7;
        this.rShape = new Coordinate[] {new Coordinate(-xl,yl),new Coordinate(xl,yl),
                                        new Coordinate(xl,-yl),new Coordinate(-xl,-yl)};

    }

    public RobotAgent( //constructor 
        int r_id,
        TrajectoryEnvelopeCoordinatorSimulation tec,
        ReedsSheppCarPlanner mp,
        Coordinate[] shape ){
        
        this.robotID = r_id;
        this.tec = tec;
        this.mp = mp;
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