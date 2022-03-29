package se.oru.coordination.coordination_oru.tests.testMAS;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;

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

import se.oru.coordination.coordination_oru.MAS.TransportAgent;
import se.oru.coordination.coordination_oru.MAS.Router;
import se.oru.coordination.coordination_oru.MAS.StorageAgent;
import se.oru.coordination.coordination_oru.MAS.DrawAgent;
import se.oru.coordination.coordination_oru.MAS.Message;

public class TimeTest {

	public static void main(String[] args) throws InterruptedException {


	// Max acceleration and velocity
	double MAX_ACCEL = 10.0;
	double MAX_VEL = 20.0;

	// final ArrayList<Integer> robotsInUse = new ArrayList<Integer>();

	

	//Instantiate a trajectory envelope coordinator
	// Dont know the difference between this and icaps
	// TODO learn what this is.
	final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
	tec.setQuiet(true);

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
	final String yamlFile = "maps/test-map.yaml";
	//yamlFile = "maps/test-map.yaml";	

	//Set up a simple GUI
	BrowserVisualization viz = new BrowserVisualization();
	viz.setMap(yamlFile);
	viz.setInitialTransform(4.0, 1.0, 1.0);
	tec.setVisualization(viz);

	//If set to true, attempts to make simulated robots slow down at cusps (this is buggy, but only affects
	//the 2D simulation, not the coordination)
	tec.setUseInternalCriticalPoints(false);

	//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

	// //Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
	ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
	//rsp.setRadius(0.2);						default is 1.0
	rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
	rsp.setTurningRadius(4.0); 				//default is 1.0
	//rsp.setDistanceBetweenPathPoints(0.5); 	default is 0.5 
	rsp.setMap(yamlFile);




	System.out.println("__________________________________________________________________________________________-");


	

	//Place robots in their initial locations (looked up in the data file that was loaded above)
	//tec.placeRobot(2, cell1);

    

	Pose DA1pos = new Pose(36.0, 35.0, Math.PI);
	Pose DA2pos = new Pose(36.0, 115.0, Math.PI);
	Pose DA3pos = new Pose(36.0, 135.0, Math.PI);


	Pose TA1pos = new Pose(50.0,20.0, Math.PI/2);	
	Pose goal = new Pose(36.0, 135.0, Math.PI);


	final int robotID = 1;


	tec.placeRobot(robotID, TA1pos);




	System.out.println("__________________________________________________________________________________________-");


	tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(
		MAX_ACCEL, 
		MAX_VEL, 
		tec.getTemporalResolution(), 
		tec.getControlPeriod(), 
		tec.getRobotTrackingPeriodInMillis(robotID)));


	System.out.println("__________________________________________________________________________________________-");

	// Motion planner
	tec.setMotionPlanner(robotID, rsp);

	
	System.out.println("__________________________________________________________________________________________-");

	rsp.setStart(TA1pos);
	rsp.setGoals(goal);
	if (!rsp.plan()) throw new Error ("No path between " + "current_pos" + " and " + goal);
	PoseSteering[] path = rsp.getPath();
	System.out.println("__________________________________________________________________________________________-");

	System.out.println("length--->" + path.length);

	double accumulatedDist = 0.0;
	for (int i=0; i< path.length-1; i++){
		Pose p1 = path[i].getPose();
		Pose p2 = path[i+1].getPose();

		double deltaS = p1.distanceTo(p2);
		System.out.println("deltaS-->" + deltaS);

		accumulatedDist += deltaS;
	}
	System.out.println("__________________________________________________________________________________________-");

	System.out.println("calculated length -----> " + accumulatedDist);
	for (int i =1; i<=10; i++){
		double vel = 0.06 + (double)(i)*0.002; //0.068
		double time = vel * accumulatedDist;
		System.out.println("vel = "+ vel + " time ----> " + time);
	}

	System.out.println("__________________________________________________________________________________________-");


	tec.addMissions(new Mission(robotID, path));

	System.out.println("starting timer:::");
	long before = System.currentTimeMillis();  
	while (true) {

		if (tec.getRobotReport(robotID).getPose().distanceTo(goal) < 0.1) break;
	}
	long after = System.currentTimeMillis(); 
	System.out.println("stopping timer:::");

	long diff = after - before;
	double secs = (double)(diff)/1000.0;
	


	//double timeInSecs = (after - before)/1000;
	System.out.println("real time to do task = " + secs);
	

	System.out.println("__________________________________________________________________________________________-");

	





	// for time testing::
	/*

	int currentPathIndex = -1;
		double accumulatedDist = 0.0;
		Trajectory traj = te.getTrajectory();
		Pose[] poses = traj.getPose();
		for (int i = 0; i < poses.length-1; i++) {
			double deltaS = poses[i].distanceTo(poses[i+1]);
			accumulatedDist += deltaS;

	*/

}

}