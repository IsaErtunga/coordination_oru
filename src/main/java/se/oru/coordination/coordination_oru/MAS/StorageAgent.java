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
import se.oru.coordination.coordination_oru.MAS.MessagingSystem;


public class StorageAgent {
    protected int robotID;
    protected MessagingSystem ms;
    protected Pose startPose;
    protected double capacity;  // capacity of storage = max ore it can store in TONS
    protected double amount;    // the current amount it has stored in TONS

    public boolean beingUsed = false;

    public ArrayList<Message> inbox = new ArrayList<Message>();
    public ArrayList<Message> outbox = new ArrayList<Message>();


    public StorageAgent(int id){this.robotID = id;}     // for testing

    public StorageAgent(int r_id, MessagingSystem ms, double capacity, Pose startPos ){
        
        this.robotID = r_id;
        this.ms = ms;
        this.capacity = capacity;
        this.amount = 0;
        this.startPose = startPos;

        this.ms.enterMessageSystem(this.robotID);
    }

    public void addOre(double ore){
        
        if (this.amount + ore <= this.capacity && ore>0) this.amount = this.amount + ore;
    }

    public double dumpOre(double reqAmount){
        double ret = 0.0;
        
        if (reqAmount > 0){
            if (this.amount - reqAmount < 0){
                ret = this.amount;
                this.amount = 0.0;
            }
            else{
                ret = reqAmount;
                this.amount = this.amount - reqAmount;
            } 
        }
        return ret;
    }

    public void communicateState(){
        // coms

    }

    public void listener(){
        

    }

}
