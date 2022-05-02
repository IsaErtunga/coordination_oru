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
import se.oru.coordination.coordination_oru.MAS.NewMapData;

public class NewMapTesting {

	public static void main(String[] args) throws InterruptedException {

	final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(20.0,20.0);
	//tec.setBreakDeadlocks(true, true, true);
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
	final String yamlFile = "maps/MineMap2Block.yaml";


	//Set up a simple GUI
	BrowserVisualization viz = new BrowserVisualization();
	viz.setMap(yamlFile);
	viz.setInitialTransform(2.0, 1.0, 1.0); // good for MineMap2Block (i think)
	tec.setVisualization(viz);
	tec.setUseInternalCriticalPoints(false);

    final long startTime = System.currentTimeMillis();
	
    												//		ROUTER THREAD
	Router router = new Router();
	Thread t3 = new Thread() {
		public void run() {
			router.run();
		}
	};
	t3.start();

	//================= SIMULATION SETTINGS ======================
	NewMapData MAP_DATA = new NewMapData();
	//================= SIMULATION SETTINGS ======================

	//================= MOTION PLANNERS ======================
	ReedsSheppCarPlanner motionPlannerSA = new ReedsSheppCarPlanner();
	motionPlannerSA.setFootprint(MAP_DATA.getAgentSize(4));
	motionPlannerSA.setTurningRadius(0.5); 				//default is 1.0
	motionPlannerSA.setDistanceBetweenPathPoints(0.5); 	//default is 0.5 
	motionPlannerSA.setMap(yamlFile);
	ReedsSheppCarPlanner motionPlannerDA = new ReedsSheppCarPlanner();
	motionPlannerDA.setFootprint(MAP_DATA.getAgentSize(2));
	motionPlannerDA.setTurningRadius(0.5); 				//default is 1.0
	motionPlannerDA.setDistanceBetweenPathPoints(0.5); 	//default is 0.5 
	motionPlannerDA.setMap(yamlFile);
	//================= MOTION PLANNERS ======================

	//================= PATH STORAGE ======================
	HashMap<String, PoseSteering[]> pathStorage = new HashMap<String, PoseSteering[]>();
	//================= PATH STORAGE ======================

	
	int[] TAs = new int[]{};
	int[] DAs = new int[]{};
	int nrOfStorages = 2;
	int[] TTAs = new int[]{9401, 9402};

	boolean spawnSAblock1 = false;
	boolean spawnSAblock2 = false;
	boolean spawnSAbaseLvl = true;

	for (final int agentID : DAs){
		try { Thread.sleep(500); }
		catch (InterruptedException e) { e.printStackTrace(); }
		Thread t = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);	

				DrawAgent DA = new DrawAgent(agentID, router, MAP_DATA, startTime, motionPlannerDA );
				DA.listener();
			}
		};
		t.start();
		
	}

	for (final int agentID : TAs){
		try { Thread.sleep(500); }
		catch (InterruptedException e) { e.printStackTrace(); }
		Thread t = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);	

				TransportAgent r = new TransportAgent( agentID, tec, MAP_DATA, router, startTime, yamlFile );
				r.start();

			}
				
		};
		t.start();
	}

	int[] block1  = new int[]{1301, 1302};
	int[] block2  = new int[]{2301, 2302};
	int[] baseLvl = new int[]{9301, 9302};

	for (int index= 0; index< nrOfStorages; index++){
		final int i = index;
		OreState oreState = new OreState(MAP_DATA.getCapacity(block1[i]), MAP_DATA.getStartOre(block1[i]));

		if ( spawnSAblock1 ){			// spawning SA on block 1
			try { Thread.sleep(500); }
			catch (InterruptedException e) { e.printStackTrace(); }
			Thread t = new Thread() {
				@Override
				public void run() {
					this.setPriority(Thread.MAX_PRIORITY);	
	
					StorageAgent SA = new StorageAgent(block1[i], router, startTime, MAP_DATA, oreState, pathStorage, motionPlannerSA);
					SA.start();
				}
			};
			t.start();
		}

		if ( spawnSAblock2 ){			// spawning SA on block 2
			try { Thread.sleep(500); }
			catch (InterruptedException e) { e.printStackTrace(); }
			Thread t = new Thread() {
				@Override
				public void run() {
					this.setPriority(Thread.MAX_PRIORITY);	
	
					StorageAgent SA = new StorageAgent(block2[i], router, startTime, MAP_DATA, oreState, pathStorage, motionPlannerSA);
					SA.start();
	
				}
					
			};
			t.start();
		}

		if ( spawnSAbaseLvl ){			// spawning SA on base lvl
			try { Thread.sleep(500); }
			catch (InterruptedException e) { e.printStackTrace(); }
			Thread t = new Thread() {
				@Override
				public void run() {
					this.setPriority(Thread.MAX_PRIORITY);	
	
					StorageAgent SA = new StorageAgent(baseLvl[i], router, startTime, MAP_DATA, oreState, pathStorage, motionPlannerSA);
					SA.start();
				}
			};
			t.start();
		}
	}

	for (final int agentID : TTAs){
		try { Thread.sleep(500); }
		catch (InterruptedException e) { e.printStackTrace(); }
		Thread t = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);	

				TransportTruckAgent TTA = new TransportTruckAgent( agentID, tec, MAP_DATA, router, startTime, yamlFile, pathStorage);
				TTA.start();

			}
				
		};
		t.start();
	}

}

}