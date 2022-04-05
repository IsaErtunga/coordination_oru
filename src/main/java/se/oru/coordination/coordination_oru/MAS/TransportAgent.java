package se.oru.coordination.coordination_oru.MAS;

import java.util.ArrayList;
import java.util.Arrays;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;


public class TransportAgent extends CommunicationAid{
    protected TrajectoryEnvelopeCoordinatorSimulation tec;
    protected ReedsSheppCarPlanner mp;

    protected Coordinate[] rShape;
    protected Pose startPose;
    protected final int oreCap = 15;

    protected TimeSchedule timeSchedule;
    protected long startTime;

    public ArrayList<Message> missionList = new ArrayList<Message>();


    public TransportAgent(int id){this.robotID = id;}   // for testing

    public TransportAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
                        ReedsSheppCarPlanner mp, Pose startPos, Router router){}

    public TransportAgent(  int r_id, TrajectoryEnvelopeCoordinatorSimulation tec,
                        ReedsSheppCarPlanner mp, Pose startPos, Router router, long startTime){
            
                            System.out.println("#######################");
                            System.out.println(r_id +" -- constructor");
        
        this.robotID = r_id;
        this.tec = tec;
        this.mp = mp;
        this.startPose = startPos;
        this.startTime = startTime;

        this.timeSchedule = new TimeSchedule(startPos);

        // enter network and broadcast our id to others.
        router.enterNetwork(this);

        String type = "hello-world";
        this.sendMessage(new Message(this.robotID, type, ""), true);
                
        double xl = 5.0;
	    double yl = 3.7;
        this.rShape = new Coordinate[] {new Coordinate(-xl,yl),new Coordinate(xl,yl),
                                        new Coordinate(xl,-yl),new Coordinate(-xl,-yl)};

    }


    protected double getTime(){
        //System.out.println(this.robotID+"\ttime---> "+(System.currentTimeMillis() - this.startTime));
        long diff = System.currentTimeMillis() - this.startTime;
        return (double)(diff)/1000.0;
    }

    protected double getNextTime(){
        double nextTime = this.timeSchedule.getNextStartTime();
        return nextTime == -1.0 ? this.getTime() : nextTime;
    }


    /**
     * 
     */
    public void start(){
        TransportAgent This = this;

        This.addRobotToSimulation();

        Thread listenerThread = new Thread() {
            public void run() {
                This.listener();
            }
        };
        listenerThread.start();

        try { Thread.sleep(2000); }
        catch (InterruptedException e) { e.printStackTrace(); }

        Thread stateThread = new Thread() {
            public void run() {
                This.initialState();
            }
        };
        stateThread.start();

        this.executeTasks();
        //try { Thread.sleep(5000); }
        //catch (InterruptedException e) { e.printStackTrace(); }

    }

    /**
     * 
     */
    public void addRobotToSimulation(){
        // add robot to simulation prep...
        double MAX_ACCEL = 10.0;
	    double MAX_VEL = 20.0;

        this.tec.setForwardModel(this.robotID, new ConstantAccelerationForwardModel(
            MAX_ACCEL, 
            MAX_VEL, 
            this.tec.getTemporalResolution(), 
            this.tec.getControlPeriod(), 
            this.tec.getRobotTrackingPeriodInMillis(this.robotID)));

        this.tec.setFootprint(this.robotID, this.rShape);

        this.tec.placeRobot(this.robotID, this.startPose);

        // Motion planner
        tec.setMotionPlanner(this.robotID, this.mp);

    }


    /**
     * 
     * @param task
     * @return
     */
    protected Boolean isMissionDone (Task task) {
        // Distance from robots actual position to task goal pose
        return this.tec.getRobotReport(this.robotID).getPose().distanceTo(task.mission.getToPose()) < 0.5;
    }

    /**
     * Function enabling TA to execute the tasks in its schedule. 
     * TODO Add functionality for knowing if robot managed to complete a task or not. 
     */
    protected void executeTasks () {
        while (true) {
            // Execute task while its schedule is not empty
            if (this.timeSchedule.getSize() > 0) {
                /* SCHEDULE: Only execute task if: 
                    * It is time to perform task. (if not wait, TODO do not just wait or something..)
                    * The task is marked as active.

                    * IF we start performing task long after startTime:
                        - update schedule to the delay
                    *   - inform receiving-end of the delay
                    * TODO: Calculate if we go over endTime.
                */
                Task task = this.timeSchedule.pop();
                if (task == null) {
                    continue;
                }
                this.tec.addMissions(task.mission);
                
                while (isMissionDone(task) == false) {
                    // not done yet
                    this.sleep(500);
                }
                //done

                // if robot managed to complete task 
                String oreChange = task.partner<10000 ? Integer.toString(this.oreCap) : Integer.toString(-this.oreCap);

                Message doneMessage = new Message(this.robotID, task.partner, "inform", task.taskID + "," + "done" + "," + Double.toString(task.ore));
                this.sendMessage(doneMessage, false);

                // if robot didn't manage to complete task
            }
            try { Thread.sleep(500); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
    }

    protected void initialState() {
        while (true) {
            // start CNP with DA
            /* SCHEDULE:
                * if endState of the last task has no ore, plan mission with DA 
                * (when robot start endState = {[time=0 : ore=0]})
                * Send time of when it can start mission to DrawAgent
             */
            // SHEDULE: Message bestOffer = this.offerService(double startTime);
            double oreLevelThreshold = 0;
            if (this.timeSchedule.checkEndStateOreLvl() > oreLevelThreshold) {
                System.out.println(this.robotID + "\tthis.timeSchedule.checkEndStateOreLvl() > oreLevelThreshold ----> " + (this.timeSchedule.checkEndStateOreLvl() > oreLevelThreshold));
                // We only create an auction if ore level is lower than treshold
                // Possibly bad to check every iteration
                this.sleep(1000);
                continue;
            }

            // SCHEDULE: Send when it can start. 
            Message bestOffer = this.offerService(this.getNextTime());
            
            if (bestOffer.isNull){ // if we got no offers from auction we sleep and try again
                this.sleep(100);
                continue;
            }

            // SCHEDULE: Receive offer message from DrawAgent. 
            // body -> int taskID & int offerVal & pos & startTime & endTime
            String[] msgParts = this.parseMessage(bestOffer, "", true);


            // create task ========================================================

            // queue mission to DA
            // TODO move to function
        
            
            double[] startCoordinates = Arrays.stream(msgParts[2].split(" "))
                                        .mapToDouble(Double::parseDouble).toArray();
            double[] endCoordinates = Arrays.stream(msgParts[3].split(" "))
                                        .mapToDouble(Double::parseDouble).toArray();

            Pose start = new Pose(startCoordinates[0], startCoordinates[1], startCoordinates[2]);
            Pose goal = new Pose(endCoordinates[0], endCoordinates[1], endCoordinates[2]);
            
            this.mp.setStart(start);
            this.mp.setGoals(goal);
            
            if (!this.mp.plan()) throw new Error ("No path between " + "current_pos" + " and " + goal);
            PoseSteering[] path = this.mp.getPath();
    
            // TODO Function that creates task based on offer message. 
            Task task = new Task(Integer.parseInt(msgParts[0]), bestOffer.sender,
                                 new Mission(this.robotID, path), true, this.oreCap, 
                                Double.parseDouble(msgParts[4]), Double.parseDouble(msgParts[5]), start, goal);
            // ===========================================================================

            // SCHEDULE: Add into schedule according to time.
            this.timeSchedule.add(task);
            this.timeSchedule.printSchedule();

            // wait for SA mission to be added
            // while (!this.schedule.isLastTaskSA()){

            //     try { Thread.sleep(300); }
            //     catch (InterruptedException e) { e.printStackTrace(); }
            // }
            this.sleep(2000);

        }

    }

    /** offerService is called when a robot want to plan in a new task to execute.
     * 
     * @param robotID id of robot{@link TransportAgent} calling this
     */
 
    public Message offerService(double startTime) {
        //System.out.println(this.robotID + "=======================1");

        // Get correct receivers
        ArrayList<Integer> receivers = this.getReceivers(this.robotsInNetwork, "DRAW");

        if (receivers.size() <= 0) return new Message();
        
        System.out.println(this.robotID +"======================2");

        this.offers.clear();

        // Create and send CNP message
        int taskID = createCNPMessage(startTime, receivers);
        System.out.println(this.robotID +"======================3");
    
        //sleep 6 sec before looking at offers
        //TODO create while loop to wait either S seconds or until all agents have responded
        double before = this.getTime();    // wait for offers..
        while ( this.offers.size() < receivers.size() || this.getTime() - before <= 2.0){
            this.sleep(50);
        }
        if ( this.offers.size() < receivers.size() ){
            System.out.println(this.robotID+"\tall offers received within "+ (this.getTime() - before) +" seconds");
        }
        
        System.out.println(this.robotID +"======================4");

    
        Message bestOffer = this.handleOffers(taskID); //extract best offer

        System.out.println(this.robotID +"======================5");

        if (!bestOffer.isNull){        
            // Send response: Mission to best offer sender, and deny all the other ones.
            Message acceptMessage = new Message(robotID, bestOffer.sender, "accept", Integer.toString(taskID) );
            this.sendMessage(acceptMessage);

            receivers.removeIf(i -> i==bestOffer.sender);

            if (receivers.size() > 0){
                // Send decline message to all the others. 
                Message declineMessage = new Message(robotID, receivers, "decline", Integer.toString(taskID));
                this.sendMessage(declineMessage);

            }
        }
        return bestOffer;
    }

    /**
     * Helper function for structuring and sending a CNP message. 
     * TODO needs to be changed in various ways. And maybe moved to communicationAid.
     * @param startTime
     * @param receivers
     * @return
     */
    public int createCNPMessage(double startTime, ArrayList<Integer> receivers) {
        // broadcast message to all transport agents
        //Pose pos = new Pose(63.0,68.0, 0.0);
        
        // SCHEDULE: TODO change to lastToPose. 
        Pose start = this.timeSchedule.getLastToPose();
        // start = this.tec.getRobotReport(this.robotID).getPose();
        String startPos = this.stringifyPose(start);

        // taskID & agentID & pos & startTime 
        String body = this.robotID + this.separator + startPos + this.separator + startTime;
        Message m = new Message(this.robotID, receivers, "cnp-service", body);

        return this.sendMessage(m, true);
    }

    /**
     * Eventually remove from communicationAid
     * SCHEDULE: Check if task fits into our schedule.
     */
    public Message handleOffers(int taskID) {

        Message bestOffer = new Message();
        int offerVal = 0;
        
        // Sort offers for the best one

        for ( Message m : this.offers ){

            // SCHEDULE: Extract startTime & endTime and see if it fits into schedule
            double startTime = Double.parseDouble(parseMessage(m, "startTime")[0]);
            double endTime = Double.parseDouble(parseMessage(m, "endTime")[0]);

            if(!m.isNull && this.timeSchedule.taskPossible(startTime, endTime)) {
                String[] mParts = this.parseMessage( m, "", true); // sort out offer not part of current auction(taskID)
                if (Integer.parseInt(mParts[0]) == taskID){
                    int val = Integer.parseInt(mParts[1]);
                    if (val > offerVal) {
                        offerVal = val;
                        bestOffer = new Message(m);
                    }
                    //this.offers.remove(m);
                }
            }
        }
        //TODO make it able to choose another offer if OG one was not possible
        return bestOffer;
    }

    /** handleService is called from within a TA, when a TA did a {@link offerService}
     * @param m the message with the service
     * @param robotID the robotID of this object
     * @return true if we send offer = we expect resp.
     */
    public boolean handleService(Message m) { 
        if (m.type != "cnp-service") return false;

        String[] mParts = this.parseMessage( m, "", true);
        double[] coordinates = Arrays.stream(mParts[2].split(" ")).mapToDouble(Double::parseDouble).toArray();
        Pose SApos = new Pose(coordinates[0], coordinates[1], coordinates[2]);


        // SCHEDULE: Need to lookup schedule too see if task is possible. 
        /* SCHEDULE: offer calc will include 
            * calc path from SA to TA
            * estimate the time TA will be using the storage
            * look in schedule if there is a time window for the task to fit in. 
            * - IF true: send offer & reserve time (Insert into schedule list)
            * - else: dont send offer

            * offer message will include: taskID, offerVal, pos, startTime, endTime
        */

        //calc euclidean dist between DA -> TA, and capacity evaluation
        //TODO setStart(Pose)?
        
        Pose start = this.timeSchedule.getLastToPose();
        this.mp.setStart(start);
        this.mp.setGoals(SApos);

        if (!this.mp.plan()) throw new Error ("No path between " + "current_pos" + " and " + SApos);
        PoseSteering[] path = this.mp.getPath();

        // Extract startTime, and calculate endTime.
        double startTime = Double.parseDouble(mParts[3]);
        double endTime = this.calculateEndTime(startTime, path);
        double distEval = endTime - startTime;

        // If task is not possible
        if (this.timeSchedule.taskPossible(startTime, endTime) == false) {
            return false; 
        }

        // SCHEDULE: Create new task & and add it to schedule
        double ore = 10.0;
        Mission mission = new Mission(this.robotID, path);
        Task TAtask = new Task(Integer.parseInt(mParts[0]), m.sender, mission, false, ore, startTime, endTime, start, SApos);
        this.timeSchedule.add(TAtask);

        // TODO: Change to time. 
        
        // Create offer
        Message response = createOffer(m, mParts, distEval, this.tec.getRobotReport(this.robotID).getPose(), startTime, endTime);
    
        // räkna ut ett bud och skicka det.
        this.sendMessage(response);
        
        System.out.println(this.robotID + ", task: " + this.activeTasks.get(Integer.parseInt(mParts[0])));
        
        return true;
    }

    /**
     * Helper function for handleService to create an offer message.
     * @param message
     * @param messageParts
     * @param evaluatedDistance
     * @return
     */
    protected Message createOffer(Message message, String[] messageParts, double evaluatedDistance, Pose position, double startTime, double endTime) {
        if (evaluatedDistance <= 0.0) evaluatedDistance = 150.0; //TODO temp fix
        int offer = (int)(100.0 * 1.0 / evaluatedDistance);
        String body = messageParts[0] + this.separator + offer + this.separator +
                      this.stringifyPose(position) + this.separator + startTime + this.separator + endTime;
        return new Message(this.robotID, message.sender, "offer", body);
    } 


    /**
     * Periodically checks the inbox of a robot.
     * Reacts to different messages depending on their message type.
     * -------------------------------------------------------------
     * if type == hello-world: add sending robot to its network
     * if type == res-offer: 
     * if type == accept: see taskHandler function. 
     */
    public void listener(){
        ArrayList<Message> inbox_copy;

        while(true){
        
            synchronized(inbox){
                inbox_copy = new ArrayList<Message>(this.inbox);
                this.inbox.clear();
            }

            for (Message m : inbox_copy){
                //TODO fix new msg type 'echo' to respond to 'hello-world'.
                // this will help us remove this.activeTasks
                String[] messageParts = this.parseMessage(m, "", true);
                

                if (m.type == "hello-world"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                    this.sendMessage( new Message( m.receiver.get(0), m.sender, "echo", ""));
                }

                if (m.type == "echo"){ 
                    if ( !this.robotsInNetwork.contains(m.sender) ) this.robotsInNetwork.add(m.sender);
                }

                else if (m.type == "accept"){
                    // SCHEDULE: Change task to actuve
                    int taskID = Integer.parseInt(messageParts[0]);
                    this.timeSchedule.setTaskActive(taskID);
                }

                else if (m.type == "decline"){
                    //remove task from activeTasks
                    //SCHEDULE: remove reserved task from schedule
                    int taskID = Integer.parseInt(messageParts[0]);
                    this.timeSchedule.remove(taskID);
                }

                else if (m.type == "cnp-service"){
                    this.handleService(m);
                }

                else if (m.type == "offer"){
                    this.offers.add(m);
                }

                else if (m.type.equals(new String("inform"))) {
                    // TA informs SA when its done with a task.
                    int taskID = Integer.parseInt(messageParts[0]);
                    String informVal = this.parseMessage(m, "informVal")[0]; 
                    
                    if (informVal.equals(new String("done"))) {
                        // current solution does not receive 'done'-msg
                        //TODO add code if implementing it that way
                        this.timeSchedule.remove(taskID);

                    }
                    else if (informVal.equals(new String("status"))) {
                        /* SCHEDULE: 
                            * if TA notice it will not be done in time, we get inform->status msg
                            * update schedule with new time and check if problem
                            * if 2 mission have big overlap then send ABORT msg to later mission.
                            * else all is good.
                        */ 
                        double newEndTime = Double.parseDouble(messageParts[2]);
                        if (newEndTime > this.timeSchedule.get(taskID).endTime) {
                            this.timeSchedule.update(taskID, newEndTime);
                        }
                    }

                    else if (informVal.equals(new String("abort"))) {
                        /* SCHEDULE:
                            * remove task from schedule
                        */
                        this.timeSchedule.remove(taskID);

                    } 

                }
                
            }
        
            // System.out.println(this.robotID + " -- " + this.robotsInNetwork);
            try { Thread.sleep(100); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
    }
}
