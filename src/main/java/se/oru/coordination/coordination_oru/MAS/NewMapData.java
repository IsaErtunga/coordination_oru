package se.oru.coordination.coordination_oru.MAS;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import com.vividsolutions.jts.geom.Coordinate;


public class NewMapData {

    public Pose getPose(int robotID){
        int block = robotID / 1000;
        int type = (robotID % 1000) / 100;
        int uniqueID = (robotID % 1000) % 100;

        if (robotID == -1) return new Pose(504.0, 185.0, 3*Math.PI/2);

        if ( block == 1 ){
            if ( type == 1 ) return new Pose(100.0, 15.0 + 34 * (uniqueID-1), Math.PI);

            else if ( type == 2 ){
                if ( uniqueID == 1 ) return new Pose(116.0, 16.0, Math.PI/2); // good -- TAstart1b1 
                else if ( uniqueID == 2 ) return new Pose(116.0, 354.0, 3*Math.PI/2);  // good -- TAstart2b1
                else if ( uniqueID == 3 ) return new Pose(116.0, 193.0, 3*Math.PI/2);  // good -- TAstart3b1

            } else if ( type == 3 ){
                if ( uniqueID == 1 ) return new Pose(139.0, 121.0, 0.0); // good -- SA1b1
                else if ( uniqueID == 2 ) return new Pose(139.0, 250.0, 0.0); // good -- SA2b1
            }

        } else if ( block == 9 ){
            if ( type == 4 ){
                if ( uniqueID == 1 ) return new Pose(391.5, 17.5, Math.PI);
                if ( uniqueID == 2 ) return new Pose(411.5, 17.5, Math.PI);
                if ( uniqueID == 3 ) return new Pose(431.5, 17.5, Math.PI);

            } else if ( type == 3 ){
                if ( uniqueID == 1 ) return new Pose(313.0, 121.0, Math.PI); // ok
                if ( uniqueID == 2 ) return new Pose(313.0, 249.0, Math.PI); // ok
            }
        } 
        Pose endDA1b1 = new Pose(20.0, 15.0, Math.PI); 
        Pose startDAb1 = new Pose(100.0, 15.0, Math.PI); 
    
        if ( block == 2 ) return null;
        return null;
    }

    public Pose[] getCorners(){
        Pose corner1 = new Pose(329.0, 44.0, Math.PI/2); // ok
        Pose corner2 = new Pose(329.0, 331.0, Math.PI/2); // ok
        Pose corner3 = new Pose(494.0, 331.0, 3*Math.PI/2); // ok
        Pose corner4 = new Pose(494.0, 44.0, 3*Math.PI/2); // ok

        return null;
    }

    public double getStartOre(int robotID){
        int type = (robotID % 1000) / 100;

        if ( type == 1 ) return this.getCapacity(type);
        if ( type == 2 ) return 0.0;
        if ( type == 3 ) return 100.0;
        if ( type == 4 ) return 0.0;
        return 0.0;
    }

    public double getCapacity(int agentType){
        if ( agentType > 1000 ) agentType = (agentType % 1000) / 100;

        if ( agentType == 1 ) return 100.0;
        if ( agentType == 3 ) return 1000.0;
        if ( agentType == 2 ) return 14.0;
        if ( agentType == 4 ) return 50.0;

        return 0.0;
    }

    public double getTurningRad(int robotType){
        return 0.5;
    }

    public double getVelocity(int robotType){
        if ( robotType > 1000 ) robotType = (robotType % 1000) / 100;

        if ( robotType == 2 ) return 5.6;
        if ( robotType == 4 ) return 5.6/2;

        return -1.0;
    }

    public Coordinate[] getAgentSize(int robotType){
        double xLength = 4.0;
        double ylength = 2.8;

        if ( robotType > 1000 ) robotType = (robotType % 1000) / 100;

        if ( robotType == 2 ){ // TA size
            xLength = 4.0;
            ylength = 2.8;
        } else if ( robotType == 4 ){ // TTA size
            xLength = 5.0;
            ylength = 3.7;
        }

        return new Coordinate[] {new Coordinate(-xLength,ylength), new Coordinate(xLength,ylength),
                                 new Coordinate(xLength,-ylength), new Coordinate(-xLength,-ylength)};
    }
}
