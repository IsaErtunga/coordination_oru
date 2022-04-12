package se.oru.coordination.coordination_oru.MAS;


import java.io.File;
import java.io.IOException; 
import java.io.FileWriter;   // Import the FileWriter class

public class FilePrinter {
    private String path = "/home/parallels/";
    private String fileName = "MAS_output.txt";
    private String separator = ",";
    private FileWriter fileWriter; 

    public FilePrinter() {
        try {
            this.fileWriter = new FileWriter(this.path+this.fileName);
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
          }
    };

    public void write(Double timeStamp, double ore) {
        try {
            this.fileWriter.write(Double.toString(timeStamp) + this.separator + Double.toString(ore));
            this.fileWriter.close();
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
          }
    }

    public static void main(String[] args) {
        FilePrinter fp = new FilePrinter();
        fp.write(0.05, 20);   
    }
}
