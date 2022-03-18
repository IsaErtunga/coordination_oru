package se.oru.coordination.coordination_oru.tests.testMAS;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.sat4j.ExitCode;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class FirstTestMap {

	public static void main(String[] args) throws InterruptedException {


	// Max acceleration and velocity
	double MAX_ACCEL = 3.0;
	double MAX_VEL = 10.0;

	// final ArrayList<Integer> robotsInUse = new ArrayList<Integer>();



	//Instantiate a trajectory envelope coordinator
	// Dont know the difference between this and icaps
	// TODO learn what this is.
	final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);

	//Provide a heuristic for determining orderings thru critical sections
	tec.addComparator(new Comparator<RobotAtCriticalSection> () {
		@Override
		public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
			CriticalSection cs = o1.getCriticalSection();
			RobotReport robotReport1 = o1.getRobotReport();
			RobotReport robotReport2 = o2.getRobotReport();
			return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
		}
	});
	tec.addComparator(new Comparator<RobotAtCriticalSection> () {
		@Override
		public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
			return (o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
		}
	});

	//Provide a conservative forward model for each robot
	tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(1)));
	tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(2)));
	tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(3)));
	tec.setForwardModel(4, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(4)));

	//Define robot geometries (here, the same for all robots)

	double xl = 5.0;
	double yl = 3.7;
	Coordinate footprint1 = new Coordinate(-xl,yl);
	Coordinate footprint2 = new Coordinate(xl,yl);
	Coordinate footprint3 = new Coordinate(xl,-yl);
	Coordinate footprint4 = new Coordinate(-xl,-yl);
	tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

	//Set up infrastructure that maintains the representation
	tec.setupSolver(0, 100000000);
	//Start the thread that checks and enforces dependencies at every clock tick
	tec.startInference();

	// viz map file
	String yamlFile = null;
	yamlFile = "maps/test-map.yaml";	

	//Set up a simple GUI
	BrowserVisualization viz = new BrowserVisualization();
	viz.setMap(yamlFile);
	viz.setInitialTransform(4.0, 1.0, 1.0);
	tec.setVisualization(viz);

	//If set to true, attempts to make simulated robots slow down at cusps (this is buggy, but only affects
	//the 2D simulation, not the coordination)
	tec.setUseInternalCriticalPoints(false);

	//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

	//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
	ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
	//rsp.setRadius(0.2);						default is 1.0
	rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
	rsp.setTurningRadius(4.0); 				//default is 1.0
	//rsp.setDistanceBetweenPathPoints(0.5); 	default is 0.5 
	rsp.setMap(yamlFile);

	//Define poses for the scenario
	Pose cell1 = new Pose(10.0, 15.0, Math.PI);
	Pose cell10 = new Pose(10.0, 135.0, Math.PI);
	Pose startPoseRobot1 = new Pose(50.0,20.0, Math.PI/2);	
	Pose startPoseRobot2 = new Pose(50.0,190.0, 3*Math.PI/2);	

	//Place robots in their initial locations (looked up in the data file that was loaded above)
	tec.placeRobot(1, startPoseRobot1);
	tec.placeRobot(2, startPoseRobot2);

	ArrayList<PoseSteering[]> paths = new ArrayList<PoseSteering[]>();


	// mission for R1
	rsp.setStart(startPoseRobot1);
	rsp.setGoals(cell10);
	if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + cell10);
	PoseSteering[] pss1 = rsp.getPath();
	paths.add(pss1);

	// mission for R2
	rsp.setStart(startPoseRobot2);
	rsp.setGoals(cell1);
	if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot2 + " and " + cell1);
	PoseSteering[] pss2 = rsp.getPath();
	paths.add(pss2);

	
	Missions.enqueueMission(new Mission(1, pss1));
	Missions.enqueueMission(new Mission(2, pss2));
	
	Missions.startMissionDispatchers(tec, 1,2);

}

}