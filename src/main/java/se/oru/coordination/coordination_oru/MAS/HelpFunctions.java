package se.oru.coordination.coordination_oru.MAS;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.sat4j.minisat.orders.VarOrderHeap;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

import java.util.Arrays;
import java.util.HashMap;
import java.lang.Math;

public class HelpFunctions {

    FilePrinter fp = new FilePrinter();

    public PoseSteering[] getPath(HashMap<String, PoseSteering[]> paths, ReedsSheppCarPlanner mp, Pose from, Pose[] to){
        String pathID = String.format("%.2f",from.getX())+"," +String.format("%.2f",from.getY());

        for (Pose p : to){
            pathID += "->"+String.format("%.2f",p.getX())+"," +String.format("%.2f",p.getY());
        }


        PoseSteering[] path = null;
        if ( paths != null ) synchronized(paths){ path = paths.get(pathID); }

        if (path != null) return path;
        synchronized(mp){
            mp.setStart(from);
            mp.setGoals(to);
            if (!mp.plan()) throw new Error ("No path between " + from + " and " + to);
            path = mp.getPath();
        }
        
        if ( paths != null ) synchronized(paths){ paths.put(pathID, path); }
        return path;
    }
    public PoseSteering[] getPath(HashMap<String, PoseSteering[]> paths, ReedsSheppCarPlanner mp, Pose from, Pose to){
        Pose[] toPoses = new Pose[] {to};
        return this.getPath(paths, mp, from, toPoses);
    }
    public PoseSteering[] getPath(ReedsSheppCarPlanner mp, Pose from, Pose to){
        Pose[] toPoses = new Pose[] {to};
        return this.getPath(null, mp, from, toPoses);
    }
    
    public PoseSteering[] calculatePath(ReedsSheppCarPlanner mp, Pose from, Pose[] to){
        synchronized(mp){
            mp.setStart(from);
            mp.setGoals(to);
            if (!mp.plan()) throw new Error ("No path between " + from + " and " + to[0]);
            return mp.getPath();
        }
    }
    public PoseSteering[] calculatePath(ReedsSheppCarPlanner mp, Pose from, Pose to){
        return this.calculatePath(mp, from, new Pose[] {to});
    }


    public double calculatePathDist(PoseSteering[] path) {
        double accumulatedDist = 0.0;
      
        for (int i=0; i< path.length-1; i++) {
            Pose p1 = path[i].getPose();
            Pose p2 = path[i+1].getPose();

            double deltaS = p1.distanceTo(p2);
            accumulatedDist += deltaS;
        }

        return accumulatedDist;
    }

    public double calculateDistTime(double dist, double vel_factor){
        return dist / vel_factor;
    }
    public double calculateDistTime(double dist){
        return this.calculateDistTime(dist, 5.6);
    }

    /**
     * Helper function that gets a pose and prepares it for message.
     * @param pose
     * @return
     */
    public String stringifyPose(Pose pose) {
        return pose.getX() + " " + pose.getY() + " " + pose.getYaw();
    }

    /**
     * extract pose from string-formatted pose
     * @param string
     * @return a pose
     */
    public Pose posefyString(String s) {
        double[] coordinates = Arrays.stream(s.split(" ")).mapToDouble(Double::parseDouble).toArray();
        return new Pose(coordinates[0], coordinates[1], coordinates[2]);
    }

    /**
     * Compare robot pose to task end pose to see if Task is finished
     * @param task
     * @return
     */
    protected void waitUntilCurrentTaskComplete (TrajectoryEnvelopeCoordinatorSimulation tec, int agentID, int cycleSleepTimeMs) {
        while ( true ){
            synchronized(tec){
                if ( tec.isFree(agentID) == true ) break;
            }
            this.sleep(cycleSleepTimeMs);
        }
    }

    /**
     * Call when need to sleep
     * @param ms
     */
    public void sleep(int ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { e.printStackTrace(); }
    }

    /**
     * Returns value from Cumulative Distribution Function
     * ATM: Exponential distribution Can play around with which one we want. 
     * Value is distance. Around 65 distance = 0 in result
     * @param value
     * @return
     */
    public int calcCDF(double value, int yValue) {
        double lambda = 0.07;
        double e = 2.718;
       
        if (value <= 0) {
            return 0;
        }

        //int res = (int) (101 - Math.pow(e, lambda*value));
        int res = (int) (yValue - value);

        return res < 0 ? 0 : res;
    }

    public double concaveDecreasingFunc(double input, double yValueAt0, double refrenceMaxInput){
        if ( input <= refrenceMaxInput -1.0 ) return yValueAt0 * Math.pow(input / refrenceMaxInput -1.0, 2.0);
        return 0.0;
    }

    public double linearDecreasingComparingFunc(double input, double compareVal, double upperDiffVal, double maxReturn){
        double diff = Math.abs(input - compareVal);
        if ( diff >= upperDiffVal ) return 0.0;
        return maxReturn - ( maxReturn * diff / upperDiffVal );
    }

    public double linearIncreasingComparingFunc(double input, double compareVal, double upperDiffVal, double maxReturn){
        double diff = Math.abs(input - compareVal);
        if ( diff >= upperDiffVal ) return maxReturn;
        return ( maxReturn * diff / upperDiffVal );
    }

    public static void main(String args[]){
        HelpFunctions test = new HelpFunctions();

        //double ret = test.linearDecreasingComparingFunc(5.0, 15.0, 15.0, 500.0);
        String aaa = "14957:14.840338448196023";
        String[] parts = aaa.split("::");
        // HelpFunctions hf = new HelpFunctions();
        for ( int i=0; i<parts.length; i++ ){
            String[] updatePair = parts[i].split(":");
            int taskID = Integer.parseInt( updatePair[0] );
            double newEndTime = Double.parseDouble( updatePair[1] );
            System.out.println(taskID +"\t"+ newEndTime);
        }
    }
}
