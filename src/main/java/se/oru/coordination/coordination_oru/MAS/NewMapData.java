package se.oru.coordination.coordination_oru.MAS;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import com.vividsolutions.jts.geom.Coordinate;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.Scanner;

public class NewMapData {
    protected HashMap<String, Integer> values = new HashMap<String, Integer>();

    public void readValues() throws FileNotFoundException {
        Scanner sc = new Scanner(new File("/home/parallels/Documents/coordination_oru/experimentValues/values.csv"));
        // alexPath "/home/parallels/Documents/coordination_oru/experimentValues/values.csv"
        sc.useDelimiter(",");

        int numCols = 12;
        int counter = 0;
        String result = "";
 
        while (sc.hasNext()) {
            if (counter == numCols) break;
            int value = Integer.parseInt(sc.next());
        
            switch(counter) {
                case 1:
                    values.put("blocks", value);
                    break;
                case 2:
                    values.put("DAs", value);
                    break;
                case 3:
                    values.put("TAs", value);
                    break;
                case 4:
                    values.put("SAs", value);
                    break;
                case 5:
                    values.put("TTas", value);
                    break;
                case 6:
                    values.put("TASpeed", value);
                    break;
                case 7:
                    values.put("TTASpeed", value);
                    break;
                case 8:
                    values.put("DACap", value);
                    break;
                case 9:
                    values.put("TACap", value);
                    break;
                case 10:
                    values.put("SACap", value);
                    break;
                case 11:
                    values.put("TTACap", value);
                    break;
                default:
                    break;
              }

            counter++;
        }
        sc.close();
    }

  
    public void printValues() {
        System.out.println("_________________Parameter Values________________");
        for (String name: values.keySet()) {
            String key = name.toString();
            String value = values.get(name).toString();
            System.out.println(key + " " + value);
        }
    }


    public boolean getLowCapacityTest() {
        return false;
    }

    public double getDropMessageTest() {
        return 0.9;
    }

    public double getRobotBreakdownTestProb(){
        return 0.05;
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
                if ( uniqueID == 1 ) return new Pose(x + 106.0, 16.0,    Math.PI/2);  
                if ( uniqueID == 2 ) return new Pose(x + 106.0, 354.0, 3*Math.PI/2);  
                if ( uniqueID == 3 ) return new Pose(x + 106.0, 193.0, 3*Math.PI/2);  

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
        if ( agentType == 3 ) return this.getCapacity(agentType)/2;
        if ( agentType == 4 ) return 0.0;
        return 0.0;
    }

    public double getCapacity(int agentType){
        if ( agentType > 1000 ) agentType = (agentType % 1000) / 100;

        if ( agentType == 1 ) return 100.0;
        if ( agentType == 3 ) return 500.0;
        if ( agentType == 2 ) return 8.0;
        if ( agentType == 4 ) return 50.0;

        return 0.0;
    }

    public double[] getWeights(int agentType){
        if ( agentType > 1000 ) agentType = (agentType % 1000) / 100;

        if ( agentType == 1 ) return new double[]{1.0,1.0,1.0}; // ore, dist, congestion
        if ( agentType == 2 ) return new double[]{1.0,1.0,1.0}; // ore, dist, time

        return null;
    }

    public double getTurningRad(int robotType){
        return 0.5;
    }

    public double getVelocity(int robotType){
        if ( robotType > 1000 ) robotType = (robotType % 1000) / 100;

        if ( robotType == 2 ) return 5.6;
        if ( robotType == 4 ) return 4 * this.getVelocity(2);

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
