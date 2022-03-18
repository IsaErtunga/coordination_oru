package se.oru.coordination.coordination_oru.MAS;
import java.util.ArrayList;
import java.util.Arrays;

public class Message {
    int sender;
    ArrayList<Integer> receiver; // if empty = broadcast
    String type;
    String body;

    
    public Message(Message m){
        this(m.sender, m.receiver, m.type, m.body);
    }

    public Message(int from, String t, String b){
        this(from, new ArrayList<Integer>(), t, b);
    }

    public Message(int from, int to, String t, String b){
        this(from, new ArrayList<>(Arrays.asList(to)), t, b);
    }

    public Message(int from, ArrayList<Integer> to, String t, String b){
        this.sender = from;
        this.receiver = to;
        this.type = t;
        this.body = b;
    }

    public Message(){}
    
}
