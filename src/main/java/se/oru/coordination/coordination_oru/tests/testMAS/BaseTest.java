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
import se.oru.coordination.coordination_oru.MAS.TransportTruckAgent;
import se.oru.coordination.coordination_oru.MAS.Router;
import se.oru.coordination.coordination_oru.MAS.StorageAgent;
import se.oru.coordination.coordination_oru.MAS.DrawAgent;
import se.oru.coordination.coordination_oru.MAS.Message;

public class BaseTest {

	public static void main(String[] args) throws InterruptedException {

    final int numTransportAgents = 2;

	// Max acceleration and velocity
	double MAX_ACCEL = 10.0;
	double MAX_VEL = 20.0;

	// final ArrayList<Integer> robotsInUse = new ArrayList<Integer>();



	//Instantiate a trajectory envelope coordinator
	// Dont know the difference between this and icaps
	// TODO learn what this is.
	final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
	tec.setBreakDeadlocks(true, true, true);
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
	final String yamlFile = "maps/test-map_complete.yaml";
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
	// ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
	// //rsp.setRadius(0.2);						default is 1.0
	// rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
	// rsp.setTurningRadius(4.0); 				//default is 1.0
	// //rsp.setDistanceBetweenPathPoints(0.5); 	default is 0.5 
	// rsp.setMap(yamlFile);

    // Define robots with poses
    final int[] robotIDs = new int[numTransportAgents];
    for (int i = 0; i < numTransportAgents; i++) robotIDs[i] = i+1;




	//Define poses for the scenario
	

	//Place robots in their initial locations (looked up in the data file that was loaded above)
	//tec.placeRobot(1, TA1pos);
	//tec.placeRobot(2, cell1);

    final long startTime = System.currentTimeMillis();

	Pose DA1pos = new Pose(36.0, 35.0, Math.PI);
	Pose DA2pos = new Pose(36.0, 115.0, Math.PI);
	Pose DA3pos = new Pose(36.0, 135.0, Math.PI);
	Pose TA1pos = new Pose(50.0,20.0, Math.PI/2);	
	Pose TA2pos = new Pose(50.0,190.0, 3*Math.PI/2);	
	Pose TA3pos = new Pose(50.0,100.0, 3*Math.PI/2);
	Pose SA1pos = new Pose(63.0,68.0, 0.0);	
	Pose SA2pos = new Pose(63.0,142.0, 0.0);

    												/*		ROUTER THREAD	*/
	Router router = new Router();
	Thread t3 = new Thread() {
		public void run() {
			router.run();
		}
	};
	t3.start();


													/*		DRAW AGENT	*/
	final int[] numDraw = {10001, 10002, 10003};
	Pose[] drawPoses = { DA1pos, DA2pos, DA3pos };
	final int[] iter3 = {0};

	ReedsSheppCarPlanner mp = new ReedsSheppCarPlanner();
	mp.setFootprint(footprint1, footprint2, footprint3, footprint4);
	mp.setTurningRadius(4.0); 				//default is 1.0
	mp.setMap(yamlFile);

	for (final int i : iter3) {

		Thread drawAgentThread = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);

				DrawAgent DA = new DrawAgent(numDraw[i], router, 20.0, drawPoses[i], mp, startTime);
				DA.listener();
				
			}
		};
		drawAgentThread.start();

	}

												/*		TRANSPORT AGENT	*/
	final int[] numTransport = {1, 2};
	final int[] iter = {};
	Pose[] transportPoses = { TA1pos, TA2pos };    
	
	for (final int i : iter) {

		// Thread for each robot object
        Thread t = new Thread() {
            
            @Override
			public void run() {
                this.setPriority(Thread.MAX_PRIORITY);

				//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
				ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
				rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
				rsp.setTurningRadius(4.0); 				//default is 1.0
				rsp.setMap(yamlFile);

				TransportAgent r = new TransportAgent(numTransport[i], tec, rsp, transportPoses[i], router, startTime);
				r.start();

			}
                
		};
        t.start();
		try { Thread.sleep(3000); }
		catch (InterruptedException e) { e.printStackTrace(); }
    }

													/*		STORAGE AGENT	*/
	final int[] numStorages = {5001, 5002};
	Pose[] storagePoses = { SA1pos, SA2pos };
	final int[] iter2 = {};

	for (final int i : iter2) {

		Thread storageThread = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);

				StorageAgent SA = new StorageAgent(numStorages[i], router, 100.0, storagePoses[i], startTime);
				SA.start();
			
			}
		};
		storageThread.start();

		try { Thread.sleep(3000); }
		catch (InterruptedException e) { e.printStackTrace(); }
	}

	final int[] numTransportTruck = {15001, 15002};
	final int[] iter4 = {0};
	Pose[] transportTruckPoses = { TA1pos, TA2pos };    
	
	for (final int i : iter4) {

		// Thread for each robot object
        Thread t = new Thread() {
            
            @Override
			public void run() {
                this.setPriority(Thread.MAX_PRIORITY);

				//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
				ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
				rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
				rsp.setTurningRadius(4.0); 				//default is 1.0
				rsp.setMap(yamlFile);

				TransportTruckAgent TTA = new TransportTruckAgent(numTransportTruck[i], tec, rsp, transportTruckPoses[i], router, startTime);
				TTA.start();

			}
                
		};
        t.start();
		try { Thread.sleep(3000); }
		catch (InterruptedException e) { e.printStackTrace(); }
    }

}

}