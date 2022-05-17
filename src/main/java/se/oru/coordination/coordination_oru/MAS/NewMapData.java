package se.oru.coordination.coordination_oru.MAS;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import com.vividsolutions.jts.geom.Coordinate;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.Scanner;

public class NewMapData {
    protected HashMap<String, Integer> values = new HashMap<String, Integer>();
    public int process_id;

    protected double SACapicity;
    protected double TAcapacity = 8.0;
    protected double TTASpeed = 5.6 * 5;
    protected double[] TAWeights = new double[]{1.0, 1.0, 1.0};

    protected double dropMessageTestProb = 1.0;
    protected double breakdownTestProb = 0.0;
    protected double lowCapacityTest = 1.0;

    protected boolean dropMessageActive = false;
    protected boolean breakdownTestActive = false;
    protected boolean lowCapacityTestActive = false;

    public int scalability = 0;

    public void readValues() throws FileNotFoundException {
        Scanner sc = new Scanner(new File("/home/parallels/Projects/coordination_oru/experimentValues/values.csv"));
        sc.useDelimiter(",");

        int numCols = 8;

        for (int i = 0; i < numCols; i++) {
            if (!sc.hasNext()) break;
            int value = Integer.parseInt(sc.next());

            switch(i) {
                case 1:
                    values.put("Scalability", value);
                    break;
                case 2:
                    values.put("RobustLevel", value);
                    break;
                case 3:
                    values.put("RobustCase", value);
                    break;
                case 4:
                    values.put("efficiency", value);
                    break;
                case 5:
                    values.put("distance", value);
                    break;
                case 6:
                    values.put("ore", value);
                    break;
                case 7:
                    values.put("time", value);
                    break;
                default:
                    break;
            } 
        }
        sc.close();

        // Set all values
        this.setExperimentValues();
    }

  
    public void printValues() {
        System.out.println("_________________Parameter Values________________");
        for (String name: values.keySet()) {
            String key = name.toString();
            String value = values.get(name).toString();
            System.out.println(key + " " + value);
        }
    }

    private void setExperimentValues() {
        for (String key: values.keySet()) {
            int value = values.get(key);
            switch (key) {
                case "Scalability":
                    this.scalability = value;
                    if (value == 1) {
                        //this.TTASpeed = 5.6 * 6.8;
                        this.TAcapacity = 5.818182; // (8/11) * 8
                    } else if (value == 2) {
                        this.TAcapacity = 4.266667; // (8/15) * 8
                        //this.TTASpeed = 5.6 * 8;
                    }
                    break;
                case "RobustLevel":
                    if (value == 1) {
                        this.dropMessageTestProb = 0.98;
                        this.breakdownTestProb = 0.025;
                        this.lowCapacityTest = 0.75;
                    } else if (value == 2) {
                        this.dropMessageTestProb = 0.95;
                        this.breakdownTestProb = 0.05;
                        this.lowCapacityTest = 0.50;
                    }
                    break;
                case "RobustCase":
                    if (value == 1) {
                        this.dropMessageActive = true;
                    } else if (value == 2) {
                        this.breakdownTestActive = true;
                    } else if (value == 3) {
                        this.lowCapacityTestActive = true;
                    } 
                    break;

                case "efficiency":
                    if (value == 0) {
                        this.SACapicity = 500.0;
                    } else if (value == 1) {
                        this.SACapicity = 500.0 * 0.75;
                    } else if (value == 2) {
                        this.SACapicity = 500.0 * 0.50;
                    }
                    break;
                case "distance":
                    this.TAWeights[0] = (double)value;
                    break;
                case "ore":
                    this.TAWeights[1] = (double)value;
                    break;
                case "time":
                    this.TAWeights[2] = (double)value;
                    break;
                default:
                    break;
            }

        }
    }


    public double getLowCapacityTest() {
        if (this.lowCapacityTestActive) {
            return this.lowCapacityTest;
        } 
        return 1.0; 
        
    }

    public double getDropMessageTest() {
        if (this.dropMessageActive) {
            return this.dropMessageTestProb;
        } 
        return 1.0; 
    }

    public double getRobotBreakdownTestProb() {
        if (this.breakdownTestActive) {
            return this.breakdownTestProb;
        } 
        return 0.0;
    }

    public Pose getPose(int robotID){
        int block = robotID / 1000;
        int type = (robotID % 1000) / 100;
        int uniqueID = (robotID % 1000) % 100;

        if (robotID == -1) return new Pose(805.0, 185.0, 3*Math.PI/2);
        
        if ( block == 9 ){
            if ( type == 3 ){
                if ( uniqueID == 1 ) return new Pose(608.0, 121.0, Math.PI);//new Pose(313.0, 121.0, Math.PI); // ok
                if ( uniqueID == 2 ) return new Pose(608.0, 249.0, Math.PI);//new Pose(313.0, 249.0, Math.PI); // ok

            } else if ( type == 4 ){
                if ( uniqueID == 1 ) return new Pose(655.0 + 1*27.5, 17.5, Math.PI);
                if ( uniqueID == 2 ) return new Pose(655.0 + 2*27.5, 17.5, Math.PI);
                if ( uniqueID == 3 ) return new Pose(655.0 + 3*27.5, 17.5, Math.PI);
            }

        } else {
            block = block - 1;
            double x = 10.0 + (block *147.0); 

            if ( type == 1 ) return new Pose(x + 80.0, 16.0 + 33.8*(uniqueID-1), Math.PI);

            else if ( type == 2 ){
                if ( uniqueID == 1 ) return new Pose(x + 106.0,20.0 + 1*82.5, Math.PI/2);  
                if ( uniqueID == 2 ) return new Pose(x + 106.0,20.0 + 3*82.5, 3*Math.PI/2);  
                if ( uniqueID == 3 ) return new Pose(x + 106.0,20.0 + 2*82.5, 3*Math.PI/2);  
                if ( uniqueID == 4 ) return new Pose(x + 106.0,20.0 + 0*82.5, Math.PI/2);  
                if ( uniqueID == 5 ) return new Pose(x + 106.0,20.0 + 4*82.5, 3*Math.PI/2);  

            } else if ( type == 3){
                if ( uniqueID == 1 ) return new Pose(x + 129.0, 121.0, 0.0); 
                if ( uniqueID == 2 ) return new Pose(x + 129.0, 249.0, 0.0);
            }
        }
        return null;
    }

    public Pose[] getCorners(){
        Pose SW1 = new Pose(655.0, 17.5, Math.PI); // ok
        Pose SW2 = new Pose(630.0, 44.0, Math.PI/2); // ok
        Pose NW1 = new Pose(630.0, 331.0, Math.PI/2); // ok
        Pose NW2 = new Pose(655.0, 352.5, 0.0); // ok
        Pose NE1 = new Pose(765.0, 352.5, 0.0); // ok
        Pose NE2 = new Pose(795.0, 331.0, 3*Math.PI/2); // ok
        Pose SE1 = new Pose(795.0, 44.0, 3*Math.PI/2); // ok
        Pose SE2 = new Pose(765.0, 17.5, Math.PI); // ok
        return new Pose[]{SW1,SW2,NW1,NW2,NE1,NE2,SE1,SE2};
    }

    public double getStartOre(int agentType){
        if ( agentType > 1000 ) agentType = (agentType % 1000) / 100;

        if ( agentType == 1 ) return this.getCapacity(agentType);
        if ( agentType == 2 ) return 0.0;
        if ( agentType == 3 ) return this.getCapacity(agentType)/4;
        if ( agentType == 4 ) return 0.0;
        return 0.0;
    }

    public double getCapacity(int agentType){
        if ( agentType > 1000 ) agentType = (agentType % 1000) / 100;

        if ( agentType == 1 ) return 250.0;
        if ( agentType == 3 ) return this.SACapicity;
        if ( agentType == 2 ) return this.TAcapacity;
        if ( agentType == 4 ) return 50.0;

        return 0.0;
    }

    public double[] getWeights(int agentType){
        if ( agentType > 1000 ) agentType = (agentType % 1000) / 100;

        if ( agentType == 1 ) return new double[]{1.0,1.0,1.0}; // ore, dist, congestion
        if ( agentType == 2 ) return this.TAWeights; // ore, dist, time

        return null;
    }

    public double getTurningRad(int robotType){
        return 0.5;
    }

    public double getVelocity(int robotType){
        if ( robotType > 1000 ) robotType = (robotType % 1000) / 100;

        if ( robotType == 2 ) return 5.6;
        if ( robotType == 4 ) return this.TTASpeed;

        return -1.0;
    }

    public Coordinate[] getAgentSize(int robotType){
        double xLength = 4.0;
        double ylength = 2.8;

        if ( robotType > 1000 ) robotType = (robotType % 1000) / 100;

        if ( robotType == 2 ){ // TA size
            xLength = 4.0;
            ylength = 2.8;
        } else if ( robotType == 4 ){ // TA size
            xLength = 4.0;
            ylength = 2.8;
        }
        // else if ( robotType == 4 ){ // TTA size
        //     xLength = 5.0;
        //     ylength = 3.7;
        // }

        return new Coordinate[] {new Coordinate(-xLength,ylength), new Coordinate(xLength,ylength),
                                 new Coordinate(xLength,-ylength), new Coordinate(-xLength,-ylength)};
    }
}
