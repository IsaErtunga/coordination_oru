package se.oru.coordination.coordination_oru.MAS;

import se.oru.coordination.coordination_oru.MAS.TransportAgent;
import se.oru.coordination.coordination_oru.MAS.StorageAgent;
import se.oru.coordination.coordination_oru.MAS.Message;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class Router {

    protected int periodMili = 250;

    public HashMap<Integer, ArrayList<Message>> inboxes = new HashMap<Integer, ArrayList<Message>>();
    public HashMap<Integer, ArrayList<Message>> outboxes = new HashMap<Integer, ArrayList<Message>>();


    public void enterNetwork(TransportAgent a){
        synchronized(this.inboxes){ this.inboxes.put(a.robotID, a.inbox); }
        synchronized(this.outboxes){ this.outboxes.put(a.robotID, a.outbox); }
    }
    public void enterNetwork(TransportTruckAgent a){
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

    public void leaveNetwork(int robotID){
        synchronized(this.inboxes){ this.inboxes.remove(robotID); }
        synchronized(this.outboxes){ this.outboxes.remove(robotID); }
    }


    public void run(){
        //TODO implement protection to check if robotID exist to router

        ArrayList<Message> outputMessages = new ArrayList<Message>();

        while(true){
            //this.print();

            synchronized(this.outboxes){
                for (Map.Entry<Integer, ArrayList<Message>> t : this.outboxes.entrySet()) {
                    ArrayList<Message> agentOutbox = t.getValue();
                    synchronized(agentOutbox){
                        outputMessages.addAll(agentOutbox);
                        agentOutbox.clear();
                    }
                }
            }

            synchronized(this.inboxes){
                for (Message m : outputMessages){
                    
                    if ( m.receiver.size() <= 0 ){      // if receiver int arr size = 0: broadcast
                        for (Map.Entry<Integer, ArrayList<Message>> t : this.inboxes.entrySet()) {
                            if ( t.getKey() == m.sender ) continue;
                            ArrayList<Message> agentInbox = this.inboxes.get(t.getKey());
                            synchronized(agentInbox){ agentInbox.add(new Message(m.sender, t.getKey(),m.type, m.body));  }
                        }
                    }

                    else{                               // send to receiver id's
                        for (int receiver : m.receiver){
                            ArrayList<Message> agentInbox = this.inboxes.get(receiver);
                            synchronized(agentInbox){ agentInbox.add(new Message(m.sender, receiver, m.type, m.body));  }
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
        
        System.out.println("\033[0;35m"+ "###ROUTER###"+ "\033[0m");
        HashMap<Integer, ArrayList<Message>> hashMapCopy;

        synchronized(this.outboxes){
            hashMapCopy = this.copy(this.outboxes);
        }
                
        for (Map.Entry<Integer, ArrayList<Message>> t : hashMapCopy.entrySet()) {
            if ( t.getValue().size() < 1 ) continue;
            System.out.println("\033[0;35m"+"outbox r" + t.getKey() +":"+ "\033[0m");

            for (Message m : t.getValue()){
                System.out.println("\033[0;35m"+" - " + m.type + ": " + m.body + "\t to r" + m.receiver+ "\033[0m");
            }
        }

        synchronized(this.inboxes){
            hashMapCopy = this.copy(this.inboxes);
        }
          
        for (Map.Entry<Integer, ArrayList<Message>> t : hashMapCopy.entrySet()) {
            if ( t.getValue().size() < 1 ) continue;

            System.out.println("\033[0;35m"+"inbox r" + t.getKey() +":"+ "\033[0m");

            for (Message m : t.getValue()){
                System.out.println("\033[0;35m"+" - " + m.type + ": " + m.body+ "\033[0m");
            }
        }
        
    }

    public HashMap<Integer, ArrayList<Message>> copy(
        HashMap<Integer, ArrayList<Message>> original)
        {
            HashMap<Integer, ArrayList<Message>> copy = new HashMap<Integer, ArrayList<Message>>();
            for (Map.Entry<Integer, ArrayList<Message>> entry : original.entrySet())
            {
                copy.put(entry.getKey(),
                // Or whatever List implementation you'd like here.
                new ArrayList<Message>(entry.getValue()));
            }
            return copy;
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
