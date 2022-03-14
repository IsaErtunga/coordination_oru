package se.oru.coordination.coordination_oru.MAS;

public class Message {
    int sender;
    int[] receiver;
    String type;
    String body;

    public Message(int from, int[] to, String t, String b){
        this.sender = from;
        this.receiver = to;
        this.type = t;
        this.body = b;
    }

    public Message(){}
    
}
