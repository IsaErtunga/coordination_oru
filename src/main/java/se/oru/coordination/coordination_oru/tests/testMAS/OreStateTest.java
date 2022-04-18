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
import se.oru.coordination.coordination_oru.MAS.OreState;

public class OreStateTest {

	public static void main(String[] args) throws InterruptedException {
 
	
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

	double xl = 4.0;
	double yl = 2.8;
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
	final String yamlFile = "maps/map_2_blocks.yaml";
	// final String yamlFile = "maps/test-map_complete.yaml";
	// yamlFile = "maps/test-map.yaml";	

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




	//Define poses for the scenario
	

	//Place robots in their initial locations (looked up in the data file that was loaded above)
	//tec.placeRobot(1, TA1pos);
	//tec.placeRobot(2, cell1);

    final long startTime = System.currentTimeMillis();

	Pose DA1posLeft = new Pose(36.0, 75.0, Math.PI);
	Pose DA2posLeft = new Pose(36.0, 95.0, Math.PI);
	Pose DA3posLeft = new Pose(36.0, 115.0, Math.PI);
	Pose DA4posLeft = new Pose(36.0, 135.0, Math.PI);
	Pose DA5posLeft = new Pose(36.0, 155.0, Math.PI);

	Pose DA1posRight = new Pose(310.0, 55.0, 0.0);	
	Pose DA2posRight = new Pose(310.0, 75.0, 0.0);
	Pose DA3posRight = new Pose(310.0, 95.0, 0.0);
	Pose DA4posRight = new Pose(310.0, 115.0, 0.0);
	Pose DA5posRight = new Pose(310.0, 135.0, 0.0);

	Pose TA1posLeft = new Pose(50.0,20.0, Math.PI/2);
	Pose TA2posLeft = new Pose(50.0,190.0, 3*Math.PI/2);	
	Pose TA3posLeft = new Pose(50.0,100.0, 3*Math.PI/2);

	// Pose TA1posRight = new Pose(292.0,20.0, Math.PI/2);
	Pose TA1posRight = new Pose(292.0,20.0, Math.PI/2);
	Pose TA2posRight = new Pose(292.0,190.0, 3*Math.PI/2);	
	Pose TA3posRight = new Pose(292.0,100.0, 3*Math.PI/2);

	Pose SA1posLeft = new Pose(63.0,68.0, 0.0);	
	Pose SA2posLeft = new Pose(63.0,142.0, 0.0);

	Pose SA1posRight = new Pose(280.0, 68.0, Math.PI);	
	Pose SA2posRight = new Pose(280.0, 142.0, Math.PI);	

	Pose SA1posTTA = new Pose(85.0, 68.0, Math.PI);	
	Pose SA2posTTA = new Pose(85.0, 142.0, Math.PI);	

	Pose TTA1pos = new Pose(140.0, 25.0, Math.PI);
	Pose TTA2pos = new Pose(170.0, 25.0, Math.PI);	

	double SAOreCapacity = 200.0;
	double SAStartOre = SAOreCapacity/4;

    												/*		ROUTER THREAD	*/
	Router router = new Router();
	Thread t3 = new Thread() {
		public void run() {
			router.run();
		}
	};
	t3.start();

	//================= PATH STORAGE ======================
	HashMap<String, PoseSteering[]> pathStorage = new HashMap<String, PoseSteering[]>();
	//================= PATH STORAGE ======================


													/*		DRAW AGENT	*/
	final int[] numDraw = {1101, 1102, 1103, 1104, 1105, 2101, 2102, 2103, 2104, 2105};
	Pose[] drawPoses = { DA1posLeft, DA2posLeft, DA3posLeft, DA4posLeft, DA5posLeft,
						 DA1posRight, DA2posRight, DA3posRight, DA4posRight, DA5posRight };
	final int[] iter3 = {0,1,2,3,5,6,7,8};

	ReedsSheppCarPlanner mp = new ReedsSheppCarPlanner();
	mp.setFootprint(footprint1, footprint2, footprint3, footprint4);
	mp.setTurningRadius(4.0); 				//default is 1.0
	mp.setMap(yamlFile);

	for (final int i : iter3) {

		Thread drawAgentThread = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);
				DrawAgent DA = new DrawAgent(numDraw[i], router, 40.0, drawPoses[i], mp, startTime, numDraw[i] < 2000 );
				DA.listener();
				
			}
		};
		drawAgentThread.start();
		try { Thread.sleep(100); }
		catch (InterruptedException e) { e.printStackTrace(); }
	}

												/*		TRANSPORT AGENT	*/
	final int[] numTransport = {1201, 1202, 1203, 2201, 2202, 2203};
	final int[] iter = {0,1,3,4};
	Pose[] transportPoses = { TA1posLeft, TA2posLeft, TA3posLeft, TA1posRight, TA2posRight, TA3posRight };    
	for (final int i : iter) {

		// Thread for each robot object
        Thread t = new Thread() {
            
            @Override
			public void run() {
                this.setPriority(Thread.MAX_PRIORITY);

				//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
				ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
				rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
				rsp.setTurningRadius(2.0); 				//default is 1.0
				rsp.setMap(yamlFile);

				TransportAgent r = new TransportAgent( numTransport[i], tec, rsp, transportPoses[i], router, startTime );
				r.start();

			}
                
		};
        t.start();
		try { Thread.sleep(100); }
		catch (InterruptedException e) { e.printStackTrace(); }
    }

													/*		STORAGE AGENT	*/
	final int[] leftNumStorages = {1301, 1302};
	final int[] rightNumStorages = {2301, 2302};
	final int[] TTANumStorages = {9301, 9302};

	Pose[] LeftStoragePoses = { SA1posLeft, SA2posLeft };
	Pose[] RightStoragePoses = {SA1posRight, SA2posRight};
	Pose[] TTAStoragePoses = {SA1posTTA, SA2posTTA};

	final int[] iter2 = {0,1};

	for (final int i : iter2) {
		OreState oreState = new OreState(SAOreCapacity, SAStartOre);
		Thread storageThreadLeft = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);
				
				ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
				rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
				rsp.setTurningRadius(4.0); 	
				rsp.setMap(yamlFile);

				StorageAgent SA = new StorageAgent(leftNumStorages[i], router, SAOreCapacity, SAStartOre, LeftStoragePoses[i], startTime, rsp, oreState, pathStorage);
				SA.start();
			
			}
		};
		storageThreadLeft.start();
		
		Thread storageThreadRight = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);
				
				ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
				rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
				rsp.setTurningRadius(4.0); 	
				rsp.setMap(yamlFile);

				StorageAgent SA = new StorageAgent(rightNumStorages[i], router, SAOreCapacity, SAStartOre, RightStoragePoses[i], startTime, rsp, oreState, pathStorage);
				SA.start();
			
			}
		};
		storageThreadRight.start();
		/*
		Thread storageThreadTTA = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);
				
				ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
				rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
				rsp.setTurningRadius(4.0); 	
				rsp.setMap(yamlFile);

				StorageAgent SA = new StorageAgent(TTANumStorages[i], router, SAOreCapacity, SAStartOre, TTAStoragePoses[i], startTime, rsp, oreState, pathStorage);
				SA.start();
			
			}
		};
		storageThreadTTA.start();
		*/

		try { Thread.sleep(3000); }
		catch (InterruptedException e) { e.printStackTrace(); }
	}

	final int[] numTransportTruck = {9401, 9402}; 
	final int[] iter4 = {};
	Pose[] transportTruckPoses = {TTA1pos, TTA2pos};    
	
	for (final int i : iter4) {

		// Thread for each robot object
        Thread t = new Thread() {
            
            @Override
			public void run() {
                this.setPriority(Thread.MAX_PRIORITY);

				//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
				ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
				rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
				rsp.setTurningRadius(1.0); 				//default is 1.0
				rsp.setMap(yamlFile);

				TransportTruckAgent TTA = new TransportTruckAgent(numTransportTruck[i], tec, rsp, transportTruckPoses[i], router, startTime, pathStorage);
				TTA.start();

			}
                
		};
        t.start();
		try { Thread.sleep(8000); }
		catch (InterruptedException e) { e.printStackTrace(); }
    }

}

}