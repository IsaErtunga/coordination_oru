package se.oru.coordination.coordination_oru.MAS;

import se.oru.coordination.coordination_oru.MAS.RobotAgent;
import se.oru.coordination.coordination_oru.MAS.StorageAgent;
import se.oru.coordination.coordination_oru.MAS.Message;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class Router {

    protected int periodMili = 500;

    public HashMap<Integer, ArrayList<Message>> inboxes = new HashMap<Integer, ArrayList<Message>>();
    public HashMap<Integer, ArrayList<Message>> outboxes = new HashMap<Integer, ArrayList<Message>>();


    public void enterNetwork(RobotAgent a){
        synchronized(this.inboxes){ this.inboxes.put(a.robotID, a.inbox); }
        synchronized(this.outboxes){ this.outboxes.put(a.robotID, a.outbox); }
    }
    public void enterNetwork(StorageAgent a){
        synchronized(this.inboxes){ this.inboxes.put(a.robotID, a.inbox); }
        synchronized(this.outboxes){ this.outboxes.put(a.robotID, a.outbox); }
    }

    public void enterNetwork(int robotID, ArrayList<Message> inbox, ArrayList<Message> outbox){ //callable from within robot
        synchronized(this.inboxes){ this.inboxes.put(robotID, inbox); }
        synchronized(this.outboxes){ this.outboxes.put(robotID, outbox); }
    }


    public void run(){
        //TODO implement protection to check if robotID exist to router

        ArrayList<Message> outputMessages = new ArrayList<Message>();

        while(true){
            this.print();

            synchronized(this.outboxes){
                
                for (Map.Entry<Integer, ArrayList<Message>> t : this.outboxes.entrySet()) {
                    outputMessages.addAll(t.getValue());
                    t.getValue().clear();
                }
            }

            synchronized(this.inboxes){
                for (Message m : outputMessages){

                    if ( m.receiver.size() <= 0 ){      // if receiver int arr size = 0: broadcast
                        for (Map.Entry<Integer, ArrayList<Message>> t : this.inboxes.entrySet()) {
                            if ( t.getKey() == m.sender ) continue;

                            this.inboxes.get(t.getKey()).add(new Message(m.sender, t.getKey(),m.type, m.body));
                        }
                    }

                    else{                               // send to receiver id's
                        for (int receiver : m.receiver){
                            this.inboxes.get(receiver).add(new Message(m.sender, receiver, m.type, m.body)); 
                        }
                    }
                }
            }
            
            outputMessages.clear();

            try { Thread.sleep(this.periodMili); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
    }

    private void print(){
        System.out.println("###ROUTER###");

        synchronized(this.outboxes){
                
            for (Map.Entry<Integer, ArrayList<Message>> t : this.outboxes.entrySet()) {
                System.out.println("outbox r" + t.getKey() +":");

                for (Message m : t.getValue()){
                    System.out.println(" - " + m.type + ": " + m.body);
                }
            }
        }
        synchronized(this.inboxes){
                
            for (Map.Entry<Integer, ArrayList<Message>> t : this.inboxes.entrySet()) {
                System.out.println("inbox r" + t.getKey() +":");

                for (Message m : t.getValue()){
                    System.out.println(" - " + m.type + ": " + m.body);
                }
            }
        }
    }

    /* ####################### TEST ####################### */

    public static void main(String args[]){

        /*
        RobotAgent r = new RobotAgent(0);
        StorageAgent s = new StorageAgent(1);
        ArrayList<Integer> aaa = new ArrayList<Integer>();
        aaa.add(0);
        s.outbox.add(new Message(1, aaa, "type-storage","body-storage"));

        Router router = new Router();

        router.enterNetwork(r);
        router.enterNetwork(s);

        Thread t = new Thread() {
			public void run() {
                int i =0;
				while (true) {
                    i++;
                    ArrayList<Integer> aaa = new ArrayList<Integer>();
                    aaa.add(0);
                    aaa.add(1);

                    synchronized(r.outbox){ r.outbox.add(new Message(0,aaa,"type-trans","body-trans-"+i)); }
					

                    try { Thread.sleep(2000); }
                    catch (InterruptedException e) { e.printStackTrace(); }
                }
            }
        };
        t.start();

        router.run();
    */

    ArrayList<Integer> receiver = new ArrayList<>(Arrays.asList(1,3));
    ArrayList<Integer> receiver2 = new ArrayList<Integer>(3);

    System.out.println(receiver2);
    }
    
    
    /* ####################### TEST ####################### */

}
