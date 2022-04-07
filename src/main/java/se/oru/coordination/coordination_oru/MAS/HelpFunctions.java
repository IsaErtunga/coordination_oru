package se.oru.coordination.coordination_oru.MAS;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;

public class HelpFunctions {



    /**
     * Helper function to calculate endTime based on startTime and the path it took. 
     * @param startTime
     * @param path
     * @return
     */
    public double calculateEndTime(double startTime, PoseSteering[] path) {
        double accumulatedDist = 0.0;
      
        for (int i=0; i< path.length-1; i++) {
            Pose p1 = path[i].getPose();
            Pose p2 = path[i+1].getPose();

            double deltaS = p1.distanceTo(p2);
            accumulatedDist += deltaS;
        }

        double vel = 0.068;
        double estimatedPathTime = vel * accumulatedDist;
        double endTime = startTime + estimatedPathTime;
        return endTime;
  }
    
}
