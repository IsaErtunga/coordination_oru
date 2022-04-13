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
    private double oreCapacity;
    
    /**
     * constructor that generates an initial state with time--> -1.0
     * and adds the oreCapacity to its attributes.
     * @param startOre the amount of ore at start.
     * @param oreCapacity the maximum ore this agent can hold.
     */
    public OreState(double oreCapacity, double startOre){
        this.oreCapacity = oreCapacity;
        addState( createState(-1.0, startOre) );
    }

    /**
     * fetches a state.
     * @param time the time of the state fetched
     * @param ore the oreChange at the state fetched (in case multiple states exist at time)
     * @return the state if found, else null.
     */
    public State getState(double time, double ore){
        for ( State s : this.oreStateArray ){
            if (s.t == time && s.deltaOre == ore) return s;
        }
        return null; // no state found
    }

    /**
     * it gets the oreState at a given time. how much ore will the agent have at the 
     * given time.
     * @param time the time of the agent.
     * @return the ore at that time
     */
    public double getStateAtTime(double time){
        int size = this.oreStateArray.size();
        
        State s = this.oreStateArray.get(size-1);
        for ( int i=size-1; i > 0; i-- ){
            s = this.oreStateArray.get(i);
            if ( s.t < time ) break;
        }
        return s.currOre;
    }

    /**
     * it retrives the last oreState in oreStateArray
     * @return double with the ore at the last state
     */
    public double getLastOreState(){
        return this.oreStateArray.get(this.oreStateArray.size()-1).currOre;
    }

    /**
     * will add an state to the oreStateArray. if two states have the same time,
     * it will put the state with the highest delaOre before. this is an optimistic 
     * approch. After adding the new state {@link update} is set correct ore value
     * in each state.
     * @param s a State that is to be added to the array.
     */
    public void addState(State s){
        int index = 0;

        if ( this.oreStateArray.size() > 0){
            for ( State curr : this.oreStateArray ){
                if ( curr.t < s.t) index++; // if time of s is after
                else if ( curr.t == s.t ){  // if time of s is same as curr

                    // we add it so there is minimum inconsistencies
                    if ( !(curr.currOre + s.deltaOre > this.oreCapacity) ) index++;
                    else if ( curr.deltaOre > s.deltaOre ) index++;
                }
                else break; //break when we hit a state with time later that curr
            }
        }
        this.oreStateArray.add(index, s);   // add at index
        update();
    }
    /* same with as above but with time, ore parameters */
    public void addState(double time, double oreChange){
        addState(createState(time, oreChange));
    }

    /**
     * removes a state from the oreStateArray.
     * Uses {@link update} if remove was successful.
     * @param s the state to be removed
     * @return true if successfully removed, else false
     */
    public boolean removeState(State s){
        boolean result = this.oreStateArray.remove(s);
        if ( result ) update();
        return result;
    }
    /* same with as above but with time, ore parameters */
    public boolean removeState(double time, double ore){
        return removeState(getState(time, ore));
    }

    /**
     * creates a state given parameters. this state does not give value to currOre
     * @param time time of state
     * @param ore deltaOre at state
     * @return the state created
     */
    public State createState(double time, double ore){
        State s = new State();
        s.t = time;
        s.deltaOre = ore;
        return s;
    }

    /**
     * this function loops thru the oreStateArray and uses deltaOre to calculate the
     * currOre at each state.
     */
    private void update(){
        double ore = 0.0;
        for ( State s : this.oreStateArray ){
            ore = ore + s.deltaOre;
            s.currOre = ore;
        }
    }

    /**
     * will change the attributes of a state given endTime and oreChange
     * @param oldTime the old time of the state being changed
     * @param oldOreChange the old deltaOre of the state being changed
     * @param newTime the new time of the state
     * @return true if state was found and updated, false if it was not found
     */
    public boolean changeState(double oldTime, double oldOreChange, double newTime){
        State s = getState(oldTime, oldOreChange);
        if ( s == null ) return false;
        removeState(s);
        s.t = newTime; 
        addState(s);
        update();
        return true;
    }
    /* same with as above but with state as param */
    public boolean changeState(State s, double newTime){
        return this.changeState(s.t, s.deltaOre, newTime);
    }    

    /**
     * this function will retrive inconsistencies in the oreStateArray. If we have a 
     * negative currOre at any state, it will need to be fixed.
     * @return
     */
    public HashMap<Double, Double> getInconsistencies(){
        HashMap<Double, Double> fixes = new HashMap<Double, Double>();

        for (State s : this.oreStateArray){
            if ( s.currOre < 0.0 || s.currOre > this.oreCapacity) fixes.put(s.t, s.currOre);
        }
        return fixes;
    }

    public void print(String c){
        String e = "\033[0m";

        System.out.println("");
        for ( State s : this.oreStateArray ){
            System.out.println(c+"time--> "+ String.format("%.2f",s.t) +"\tore at state--> "+s.currOre+"\tdeltaOre--> "+s.deltaOre+e);
        }
    }


    public static void main(String[] args){
        OreState os = new OreState(100.0, 20.0);

        os.addState( os.createState(5.0, 15.0) );
        os.addState( os.createState(10.0, -25.0) );
        os.addState( os.createState(5.0, 10.0) );
        os.addState( os.createState(9.0, 16.0) );
        os.addState( os.createState(10.0, 40.0) );
        os.print("");

        System.out.println(os.getStateAtTime(7.0));

    }
    
}


