package se.oru.coordination.coordination_oru.MAS;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.sat4j.minisat.orders.VarOrderHeap;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import java.util.Arrays;
import java.util.HashMap;
import java.lang.Math;

public class HelpFunctions {

    FilePrinter fp = new FilePrinter();

    public PoseSteering[] getPath(HashMap<String, PoseSteering[]> paths, ReedsSheppCarPlanner mp, Pose from, Pose[] to){
        Pose finalToPose = to[to.length-1];
        String pathID = String.format("%.2f",from.getX())+"," +String.format("%.2f",from.getY())
                        + "->" +
                        String.format("%.2f",finalToPose.getX())+"," +String.format("%.2f",finalToPose.getY());


        PoseSteering[] path = null;
        if ( paths != null ) synchronized(paths){ path = paths.get(pathID); }

        if (path != null) return path;

        mp.setStart(from);
        mp.setGoals(to);
        if (!mp.plan()) throw new Error ("No path between " + from + " and " + to);

        path = mp.getPath();
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

    public double calculateDistTime(double dist){
        return dist * 0.068;
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

    public static void main(String args[]){

        // HelpFunctions hf = new HelpFunctions();
        // System.out.println(hf.calcCDF(100));

    }
}
