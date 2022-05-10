package se.oru.coordination.coordination_oru.tests.testMAS;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Arrays;
import java.util.logging.Level;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.sat4j.ExitCode;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.utility.logging.MetaCSPLogging;

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
import se.oru.coordination.coordination_oru.MAS.FilePrinter;
import se.oru.coordination.coordination_oru.MAS.Message;
import se.oru.coordination.coordination_oru.MAS.OreState;
import se.oru.coordination.coordination_oru.MAS.NewMapData;

public class NewMapTesting {

	public static void main(String[] args) throws InterruptedException {

	final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(30.0,20.0);
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

	MetaCSPLogging.setLevel(Level.OFF);

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
	double resolution = 0.1;

	//Set up a simple GUI
	BrowserVisualization viz = new BrowserVisualization();
	viz.setMap(yamlFile);
	viz.setInitialTransform(2.0, 1.0, 1.0); // good for MineMap2Block (i think)
	tec.setVisualization(viz);
	tec.setUseInternalCriticalPoints(false);

	final long startTime = System.currentTimeMillis();
												
	ArrayList<String> loggedMessages = new ArrayList<String>();		//		ROUTER THREAD
	Router router = new Router(startTime, loggedMessages);
	Thread t3 = new Thread() {
		public void run() {
			router.run();
		}
	};
	t3.start();

	// Has path. 
	FilePrinter fp = new FilePrinter(false, loggedMessages, startTime);			//		FILE PRINTER
	Thread printer = new Thread() {
		@Override
		public void run() {
			this.setPriority(Thread.MAX_PRIORITY);	
			while (true) {
				fp.logMessages();
				fp.writeValueToFile();

				try { Thread.sleep(5000); }
				catch (InterruptedException e) { e.printStackTrace(); }
			}
		}
	};
	printer.start();


	//================= SIMULATION SETTINGS ======================
	// has path
	NewMapData MAP_DATA = new NewMapData();
	try {
		MAP_DATA.readValues();
	} catch (FileNotFoundException e1) {
		e1.printStackTrace();
	}
	//MAP_DATA.printValues();
	//================= SIMULATION SETTINGS ======================

	//================= MOTION PLANNERS ======================
	
	/*
	LBTRRT is reliable 
	LBTRRT 2.0s , 4.0 BPP is ok
	RRTstar 2.0s, 4.0 BPP is better

	*/


	ReedsSheppCarPlanner mp1 = new ReedsSheppCarPlanner();
	mp1.setPlanningTimeInSecs(5.0);
	mp1.setFootprint(MAP_DATA.getAgentSize(2));
	mp1.setTurningRadius(2.0); 				//default is 1.0
	mp1.setDistanceBetweenPathPoints(2.0); 	//default is 0.5 
	mp1.setMap(yamlFile);
	ReedsSheppCarPlanner mp2 = new ReedsSheppCarPlanner();
	mp2.setFootprint(MAP_DATA.getAgentSize(4));
	mp2.setTurningRadius(2.0); 				//default is 1.0
	mp2.setDistanceBetweenPathPoints(2.0); 	//default is 0.5 
	mp2.setMap(yamlFile);
	ReedsSheppCarPlanner mp3 = new ReedsSheppCarPlanner();
	mp3.setFootprint(MAP_DATA.getAgentSize(4));
	mp3.setTurningRadius(2.0); 				//default is 1.0
	mp3.setDistanceBetweenPathPoints(2.0); 	//default is 0.5 
	mp3.setMap(yamlFile);
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

	// ============================================================
	// ============== HERE YOU ALTER THE SCENARIO =================

	// agents spawning in rep, blocks. index0 = DA's, index2 = TA's, index3 = SA's
	Integer[] block1Agents = new Integer[]{2,2,2}; 
	Integer[] block2Agents = new Integer[]{0,0,0};
	Integer[] block3Agents = new Integer[]{0,0,0};
	Integer[] block4Agents = new Integer[]{0,0,0};
	// base lvl. index0 = TTA
	int nrTTAs = 0;
	// ============================================================

	// =========== DONT CHANGE ANYTHING AFTER THIS LINE============
	ArrayList<Integer[]> blockSpawns = new ArrayList<Integer[]>();
	blockSpawns.add(block1Agents);
	blockSpawns.add(block2Agents);
	blockSpawns.add(block3Agents);
	blockSpawns.add(block4Agents);

	boolean spawnBaseLvlSA1 = nrTTAs > 0 ? true : false;
	boolean spawnBaseLvlSA2 = nrTTAs > 0 ? true : false;
	

	OreState oreState1 = new OreState(MAP_DATA.getCapacity(3), MAP_DATA.getStartOre(3));
	OreState oreState2 = new OreState(MAP_DATA.getCapacity(3), MAP_DATA.getStartOre(3));

	for ( int agentType =0; agentType<3; agentType++ ){ // spawnOrder = DA,TA,SA,TTA, for every agent type

		for ( int block = 0; block <4; block++ ){	// for every block
			int nrAgents = blockSpawns.get(block)[agentType];

			if ( agentType == 0 ){ // spawn DA
				int[] spawnOrderDA = new int[]{3,9,5,7,6,2,10,4,8,1,11};
				for ( int agent = 0; agent<nrAgents; agent++ ){ // for every agent
					try { Thread.sleep(300); }
					catch (InterruptedException e) { e.printStackTrace(); }
					int agentID = (block+1)*1000 + 100 + spawnOrderDA[agent];
					Thread t = new Thread() {
						@Override
						public void run() {
							this.setPriority(Thread.MAX_PRIORITY);	
			
							DrawAgent DA = new DrawAgent(agentID, router, MAP_DATA, startTime );
							DA.listener();
						}
					};
					t.start();
				}

			} else if ( agentType == 1 ){ //spawn TA
				int[] spawnOrderTA = new int[]{1,2,3};
				for ( int agent = 0; agent<nrAgents; agent++ ){
					try { Thread.sleep(300); }
					catch (InterruptedException e) { e.printStackTrace(); }

					int agentID = (block+1)*1000 + 200 + spawnOrderTA[agent];
					ReedsSheppCarPlanner mp;
					if ( spawnOrderTA[agent] == 1 ) mp = mp1;
					else if ( spawnOrderTA[agent] == 2 ) mp = mp2;
					else mp = mp3;

					Thread t = new Thread() {
						@Override
						public void run() {
							TransportAgent r = new TransportAgent( agentID, tec, MAP_DATA, router, startTime, mp, fp );
							r.start();
						}
					};
					t.start();
				}

			} else if ( agentType == 2 ){ //spawn SA
				int[] spawnOrderSA = new int[]{1,2};
				for ( int agent = 0; agent<nrAgents; agent++ ){
					try { Thread.sleep(300); }
					catch (InterruptedException e) { e.printStackTrace(); }
					int agentID = (block+1)*1000 + 300 + spawnOrderSA[agent];
					OreState os = spawnOrderSA[agent] == 1 ? oreState1 : oreState2;
					
					Thread t = new Thread() {
						@Override
						public void run() {
							this.setPriority(Thread.MAX_PRIORITY);	

							StorageAgent SA = new StorageAgent(agentID, router, startTime, MAP_DATA, os, pathStorage, fp);
							SA.start();
						}
					};
					t.start();
				}
			}
		}
		if ( agentType == 2 && spawnBaseLvlSA1 ){
			try { Thread.sleep(300); }
			catch (InterruptedException e) { e.printStackTrace(); }
			spawnBaseLvlSA1 = false;
			Thread t = new Thread() {
				@Override
				public void run() {
					this.setPriority(Thread.MAX_PRIORITY);	
					StorageAgent SA = new StorageAgent(9301, router, startTime, MAP_DATA, oreState1, pathStorage, fp);
					SA.start();
				}
			};
			t.start();
		}
		if ( agentType == 2 && spawnBaseLvlSA2 ){
			try { Thread.sleep(300); }
			catch (InterruptedException e) { e.printStackTrace(); }
			spawnBaseLvlSA2 = false;
			Thread t = new Thread() {
				@Override
				public void run() {
					this.setPriority(Thread.MAX_PRIORITY);	
					StorageAgent SA = new StorageAgent(9302, router, startTime, MAP_DATA, oreState2, pathStorage, fp);
					SA.start();
				}
			};
			t.start();
		}
	}

	for ( int agent=0; agent < nrTTAs; agent++){
		try { Thread.sleep(3000); }
		catch (InterruptedException e) { e.printStackTrace(); }
		final int agentID = 9400 + (agent+1);
		Thread t = new Thread() {
			@Override
			public void run() {
				this.setPriority(Thread.MAX_PRIORITY);	

				TransportTruckAgent TTA = new TransportTruckAgent( agentID, tec, MAP_DATA, router, startTime, mp3, pathStorage,fp);
				TTA.start();
			}
		};
		t.start();
	}

}

}