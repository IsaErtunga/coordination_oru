package se.oru.coordination.coordination_oru.MAS;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import java.util.Arrays;

public class HelpFunctions {


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
     * Helper function to calculate distance between to Pose objects. 
     * @param start
     * @param end
     * @return
     */
    public double calcDistance(Pose start, Pose end) {
        return start.distanceTo(end);
    }
    
}
