package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.HashMap;

public class OreState {
    class State{
        double t;
        double currOre;
        double deltaOre;

    }
    private ArrayList<State> oreStateArray = new ArrayList<State>();


    public OreState(double startOre){
        addState( createState(-1.0, startOre) );
    }


    public boolean addState(State s){
        this.oreStateArray.add(s);
        update();
        return true;
    }

    public State createState(double time, double ore){
        State s = new State();
        s.t = time;
        s.deltaOre = ore;
        return s;
    }

    private void update(){
        double ore = 0.0;
        for ( State s : this.oreStateArray ){
            ore = ore + s.deltaOre;
            s.currOre = ore;
        }
    }

    public HashMap<Double, Double> getInconsistencies(){
        HashMap<Double, Double> fixes = new HashMap<Double, Double>();

        for (State s : this.oreStateArray){
            if ( s.currOre < 0.0 ) fixes.put(s.t, s.currOre);
        }
        return fixes;
    }

    public void print(){
        System.out.println("");
        for ( State s : this.oreStateArray ){
            System.out.println("time--> "+s.t+"\tore at state--> "+s.currOre+"\tdeltaOre--> "+s.deltaOre);
        }
    }


    public static void main(String[] args){
        OreState os = new OreState(20.0);

        os.addState( os.createState(5.0, -25.0) );
        os.print();

        System.out.println(os.getInconsistencies());
    }
    
}


