package se.oru.coordination.coordination_oru.tests.testMAS;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Arrays;

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
	final String yamlFile = "maps/MineMap4Block.yaml";


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
	ReedsSheppCarPlanner TAmotionPlanner = new ReedsSheppCarPlanner();
	TAmotionPlanner.setFootprint(MAP_DATA.getAgentSize(2));
	TAmotionPlanner.setTurningRadius(2.0); 				//default is 1.0
	TAmotionPlanner.setDistanceBetweenPathPoints(0.5); 	//default is 0.5 
	TAmotionPlanner.setMap(yamlFile);
	ReedsSheppCarPlanner TTAmotionPlanner = new ReedsSheppCarPlanner();
	TTAmotionPlanner.setFootprint(MAP_DATA.getAgentSize(4));
	TTAmotionPlanner.setTurningRadius(2.0); 				//default is 1.0
	TTAmotionPlanner.setDistanceBetweenPathPoints(0.5); 	//default is 0.5 
	TTAmotionPlanner.setMap(yamlFile);
	//================= MOTION PLANNERS ======================

	//================= PATH STORAGE ======================
	HashMap<String, PoseSteering[]> pathStorage = new HashMap<String, PoseSteering[]>();
	//================= PATH STORAGE ======================

	/* good example with 2SA 3TA: 
	int[] TAs = new int[]{1201, 1202,1203};
	int[] DAs = new int[]{1103,1105,1107,1108,1109};
	*/

	/* good example with 1SA 2TA
	int[] TAs = new int[]{1201,1203};
	int[] DAs = new int[]{1102,1103,1105,1106};
	*/

	/*
	int[] TAs = new int[]{1201,1202,1203};
	int[] DAs = new int[]{1103,1105,1106,1107,1109};
	*/

	// agents spawning in rep, blocks. index0 = DA's, index2 = TA's, index3 = SA's
	int[] block1Agents = new int[]{0,0,0}; 
	int[] block2Agents = new int[]{0,0,0};
	int[] block3Agents = new int[]{0,0,0};
	int[] block4Agents = new int[]{0,0,0};
	// base lvl. index0 = TTA
	int[] baseLvlAgents = new int[]{0};

	ArrayList<Integer[]> blockSpawns = new ArrayList<Integer[]>();
	blockSpawns.add(block1Agents);
	blockSpawns.add(block2Agents);
	blockSpawns.add(block3Agents);
	blockSpawns.add(block4Agents);
	//blockSpawns.add(baseLvlAgents);


	// spawn order of agents
	//int[] spawnOrderTA = new int[]{1,2,3};
	int[] spawnOrderSA = new int[]{1,2};
	//int[] spawnOrderDA = new int[]{3,9,5,7,6,2,10,4,8,1,11};

	int blocks = Arrays.stream(block1Agents).sum();

	for ( int i=0; i<3; i++ ){ // spawnOrder = DA,TA,SA,TTA
		for ( int block = 0; block <4; block++ ){
			int nrAgents = blockSpawns.get(block)[i];

			if ( i == 0 ){ // spawn DA
				int[] spawnOrderDA = new int[]{3,9,5,7,6,2,10,4,8,1,11};
				for ( int agent = 0; i<nrAgents; i++ ){
					int agentID = (block+1)*1000 + 100 + spawnOrderDA[agent];
					Thread t = new Thread() {
						@Override
						public void run() {
							this.setPriority(Thread.MAX_PRIORITY);	
			
							DrawAgent DA = new DrawAgent(agentID, router, MAP_DATA, startTime, TAmotionPlanner );
							DA.listener();
						}
					};
					t.start();
				}

			} else if ( i == 1 ){ //spawn TA
				int[] spawnOrderTA = new int[]{1,2,3};
				for ( int agent = 0; i<nrAgents; i++ ){
					int agentID = (block+1)*1000 + 200 + spawnOrderTA[agent];
					Thread t = new Thread() {
						@Override
						public void run() {
							this.setPriority(Thread.MAX_PRIORITY);	
			
							TransportAgent r = new TransportAgent( agentID, tec, MAP_DATA, router, startTime, TAmotionPlanner );
							r.start();
						}
							
					};
					t.start();
				}


			} else if ( i == 2 ){ //spawn SA
				int[] spawnOrderTA = new int[]{1,2,3};
				for ( int agent = 0; i<nrAgents; i++ ){
					int agentID = (block+1)*1000 + 200 + spawnOrderTA[agent];
					Thread t = new Thread() {
						@Override
						public void run() {
							this.setPriority(Thread.MAX_PRIORITY);	
			
							TransportAgent r = new TransportAgent( agentID, tec, MAP_DATA, router, startTime, TAmotionPlanner );
							r.start();
						}
							
					};
					t.start();
				}
			}
		}
	}

	
	
	int[] TAs = new int[]{2201,2202};
	int[] DAs = new int[]{2101,2109};
	int[] SAs = new int[]{2301,2302}; //TODO
	int nrOfStorages = 2;
	int[] TTAs = new int[]{};

	boolean spawnSAblock1 = false;
	boolean spawnSAblock2 = true;
	boolean spawnSAbaseLvl = false;

	for (final int agentID : DAs){
		try { Thread.sleep(500); }
		catch (InterruptedException e) { e.printStackTrace(); }
		Thread t = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);	

				DrawAgent DA = new DrawAgent(agentID, router, MAP_DATA, startTime, TAmotionPlanner );
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

				TransportAgent r = new TransportAgent( agentID, tec, MAP_DATA, router, startTime, TAmotionPlanner );
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
	
					StorageAgent SA = new StorageAgent(block1[i], router, startTime, MAP_DATA, oreState, pathStorage, TTAmotionPlanner);
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
	
					StorageAgent SA = new StorageAgent(block2[i], router, startTime, MAP_DATA, oreState, pathStorage, TTAmotionPlanner);
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
	
					StorageAgent SA = new StorageAgent(baseLvl[i], router, startTime, MAP_DATA, oreState, pathStorage, TTAmotionPlanner);
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

				TransportTruckAgent TTA = new TransportTruckAgent( agentID, tec, MAP_DATA, router, startTime, TTAmotionPlanner, pathStorage);
				TTA.start();

			}
				
		};
		t.start();
	}

}

}