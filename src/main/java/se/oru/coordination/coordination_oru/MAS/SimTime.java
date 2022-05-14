package se.oru.coordination.coordination_oru.MAS;

public class SimTime {
    long time = 0;
    double TEMPORAL_RES_NOM;
    double TEMPORAL_RES;
    long startTime = 0;

    public SimTime(double temporal_res, long startTIme){
        this.TEMPORAL_RES = temporal_res;
        this.TEMPORAL_RES_NOM = 1.0/temporal_res;
        this.startTime = startTIme;
        System.out.println("constr: "+this.TEMPORAL_RES_NOM);
    }

    public void startClock(){
        while (true){
            try { Thread.sleep(10); }
            catch (InterruptedException e) { e.printStackTrace(); }
            this.time += 10;
        }
    }

    public double getTemporalRes(){
        return this.TEMPORAL_RES;
    }

    public long time(){
        return this.time - this.startTime;
    }

    public double SIM_time(){
        return (double)(this.time - this.startTime) * this.TEMPORAL_RES_NOM;
    }
    
}
