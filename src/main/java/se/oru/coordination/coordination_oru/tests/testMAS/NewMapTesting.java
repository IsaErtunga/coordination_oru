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

	// final ArrayList<Integer> robotsInUse = new ArrayList<Integer>();



	//Instantiate a trajectory envelope coordinator
	// Dont know the difference between this and icaps
	// TODO learn what this is.
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
	// final String yamlFile = "maps/test-map_complete.yaml";
	// yamlFile = "maps/test-map.yaml";	

	//Set up a simple GUI
	BrowserVisualization viz = new BrowserVisualization();
	viz.setMap(yamlFile);
	viz.setInitialTransform(2.0, 1.0, 1.0); // good for MineMap2Block (i think)

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


	double SAOreCapacity = 200.0;
	double SAStartOre = 0.0;

	
    												//		ROUTER THREAD
	Router router = new Router();
	Thread t3 = new Thread() {
		public void run() {
			router.run();
		}
	};
	t3.start();
	/*

	//================= PATH STORAGE ======================
	HashMap<String, PoseSteering[]> pathStorage = new HashMap<String, PoseSteering[]>();
	//================= PATH STORAGE ======================

	*/
										/*		DRAW AGENT	*/
	/*
	final int[] numDraw = {1101, 1102, 1103, 1104, 1105, 2101, 2102, 2103, 2104, 2105};
	Pose[] drawPoses = { DA1posLeft, DA2posLeft, DA3posLeft, DA4posLeft, DA5posLeft,
						 DA1posRight, DA2posRight, DA3posRight, DA4posRight, DA5posRight };
	final int[] iter3 = {0,1,2,3};

	ReedsSheppCarPlanner mp = new ReedsSheppCarPlanner();
	mp.setFootprint(footprint1, footprint2, footprint3, footprint4);
	mp.setTurningRadius(4.0); 				//default is 1.0
	mp.setMap(yamlFile);

	for (final int i : iter3) {

		Thread drawAgentThread = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);
				DrawAgent DA = new DrawAgent(numDraw[i], router, 100.0, drawPoses[i], mp, startTime, numDraw[i] < 2000 );
				DA.listener();
				
			}
		};
		drawAgentThread.start();
		try { Thread.sleep(100); }
		catch (InterruptedException e) { e.printStackTrace(); }
	}

	

												/*		TRANSPORT AGENT	*/

	// ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
	// rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
	// rsp.setTurningRadius(1.0); 				//default is 1.0
	// rsp.setDistanceBetweenPathPoints(2.0); 	//default is 0.5 
	// rsp.setMap(yamlFile);

	NewMapData MAP_DATA = new NewMapData();

	Pose startDAb1 = new Pose(20.0, 15.0, Math.PI); // good // 34.0 interval between drawAgents
	Pose endDAb1 = new Pose(100.0, 15.0, Math.PI); // good
	Pose spawn1b1 = new Pose(116.0, 16.0, Math.PI/2); // good
	Pose SA1b1 = new Pose(139.0, 121.0, 0.0);  // good // 129.0 interval between lower and upper SA

	Pose startP = new Pose(329.0, 44.0, Math.PI/2); 
	//Pose toP = new Pose(116.0, 16.0, Math.PI/2);

	//================= PATH STORAGE ======================
	HashMap<String, PoseSteering[]> pathStorage = new HashMap<String, PoseSteering[]>();
	//================= PATH STORAGE ======================

	
	int[] TAs = new int[]{};
	int[] DAs = new int[]{};
	int nrOfStorages = 1;
	int[] TTAs = new int[]{9401};

	boolean spawnSAblock1 = false;
	boolean spawnSAblock2 = false;
	boolean spawnSAbaseLvl = true;

	for (final int agentID : DAs){
		Thread t = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);	

				DrawAgent DA = new DrawAgent(agentID, router, MAP_DATA, startTime, yamlFile );
				DA.listener();
			}
		};
		t.start();
	}

	for (final int agentID : TAs){
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
			Thread t = new Thread() {
				@Override
				public void run() {
					this.setPriority(Thread.MAX_PRIORITY);	
	
					StorageAgent SA = new StorageAgent(block1[i], router, startTime, MAP_DATA, yamlFile, oreState, pathStorage);
					SA.start();
				}
			};
			t.start();
		}
		if ( spawnSAblock2 ){			// spawning SA on block 2
			Thread t = new Thread() {
				@Override
				public void run() {
					this.setPriority(Thread.MAX_PRIORITY);	
	
					StorageAgent SA = new StorageAgent(block2[i], router, startTime, MAP_DATA, yamlFile, oreState, pathStorage);
					SA.start();
	
				}
					
			};
			t.start();
		}
		if ( spawnSAbaseLvl ){			// spawning SA on base lvl
			Thread t = new Thread() {
				@Override
				public void run() {
					this.setPriority(Thread.MAX_PRIORITY);	
	
					StorageAgent SA = new StorageAgent(baseLvl[i], router, startTime, MAP_DATA, yamlFile, oreState, pathStorage);
					SA.start();
				}
			};
			t.start();
		}
	}

	for (final int agentID : TTAs){
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
	

	
	/*
	for ( int i=0; i<nrTAblock1; i++ ){
		// Thread for each robot object
		Thread t = new Thread() {
					
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);	

				TransportAgent r = new TransportAgent( 1201, tec, rsp, startP, router, startTime );
				r.start();

			}
				
		};
		t.start();
	}
	
	try { Thread.sleep(1000); }
	catch (InterruptedException e) { e.printStackTrace(); }
	*/
	/*
	rsp.setStart(TAstart2b1);
	rsp.setGoals(mDat.getPose(1104));
	if (!rsp.plan()) throw new Error ("No path between " + TAstart2b1 + " and " + mDat.getPose(1104));
	PoseSteering[] path = rsp.getPath();
	double accumulatedDist = 0.0;
	for (int i=0; i< path.length-1; i++) {
		Pose p1 = path[i].getPose();
		Pose p2 = path[i+1].getPose();

		double deltaS = p1.distanceTo(p2);
		accumulatedDist += deltaS;
	}
	double robotVel = 5.6; // 5.6m/s = 20km/s
	double estTime = accumulatedDist/robotVel;
	System.out.println("path points-->"+path.length+"\t path dist est-->"+ accumulatedDist+"\t estTime-->"+estTime);
	System.out.println("euclidean dist-->"+(startP.distanceTo(toP)));
	tec.addMissions(new Mission(1202, path));
	*/
													/*		STORAGE AGENT	*/
													/*
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
		/*
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
		

		try { Thread.sleep(100); }
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
	*/

}

}