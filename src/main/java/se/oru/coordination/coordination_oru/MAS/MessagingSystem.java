package se.oru.coordination.coordination_oru.MAS;

import java.util.HashMap;
import java.util.Map;
import java.util.ArrayList;

public class MessagingSystem {

    public HashMap<Integer, ArrayList<String>> messageBoard = new HashMap<Integer, ArrayList<String>>();
    public int verbose = 1;


    public void enterMessageSystem(int robotID){
        // when a new robot enters the system it calls this function to enter the messaging system
        
        synchronized(this.messageBoard){
            this.messageBoard.put(robotID, new ArrayList<String>());
        }

        if (this.verbose !=0){
            System.out.println("new robot: id " + robotID);
            if (this.verbose > 1) System.out.println(this.messageBoard);
        }
    }

    public void leaveMessageSystem(int robotID){
        // a robot leaves the messaging system
        synchronized(this.messageBoard){
            this.messageBoard.remove(robotID);
        }

        if (this.verbose !=0){
            System.out.println("removed robot: id " + robotID);
            if (this.verbose > 1) System.out.println(this.messageBoard);
        }
        
    }

    public String getMessage(int robotID){
        // a robot checks if it has got a new message, if not 'NULL' is returned
        String ret_msg = "NULL";

        if (this.verbose !=0){
            System.out.println("getMsg: id " + robotID);
            if (this.verbose > 1) System.out.println(this.messageBoard);
        }

        synchronized(this.messageBoard){
            if (this.messageBoard.get(robotID).size() > 0) ret_msg = this.messageBoard.get(robotID).remove(0);
        }

        return ret_msg;
    }

    public void putMessage(int receiverID, String msg){
        // a robot send a message to another robot
        synchronized(this.messageBoard){
            this.messageBoard.get(receiverID).add(msg);
        }

        if (this.verbose !=0){
            System.out.println("putMsg: to " + receiverID);
            if (this.verbose > 1) System.out.println(this.messageBoard);
        }
    }

    public void multiSendMessage(int[] receiverIDs, String msg){
        // a robot send a message to a group of robots
        synchronized(this.messageBoard){
            for (int id : receiverIDs) this.messageBoard.get(id).add(msg);
        }

        if (this.verbose !=0){
            System.out.println("multiSend: to ids " + receiverIDs);
            if (this.verbose > 1) System.out.println(this.messageBoard);
        }
    }

    public void broadcastMessage(int senderID, String msg){
        // a robot send a message to all other robots
        synchronized(this.messageBoard){
            for (Map.Entry<Integer, ArrayList<String>> agentMsgListEntry : this.messageBoard.entrySet()) {
                if (senderID == agentMsgListEntry.getKey()) continue;
                agentMsgListEntry.getValue().add(msg);
            }
        }

        if (this.verbose !=0){
            System.out.println("broadcast: from " + senderID);
            if (this.verbose > 1) System.out.println(this.messageBoard);
        }
    }

}


