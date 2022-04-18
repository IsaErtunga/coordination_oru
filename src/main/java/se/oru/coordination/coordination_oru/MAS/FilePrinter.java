package se.oru.coordination.coordination_oru.MAS;


// Java Program to Write Into a File
// using writeString() Method
 
// Importing required classes
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
 
// Main class
public class FilePrinter {
    protected String separator = ",";
    protected ArrayList<Integer> robots = new ArrayList<Integer>();

    /**
     * For writing storage state into file
     * @param time
     * @param ore
     * @throws IOException
     */
    protected void write(double time, double ore, int robotID) throws IOException {
        Path path = Path.of("/home/parallels/" + "OreState" + robotID + ".csv");
        String content = time + this.separator + ore + "\n";
        if (Files.exists(path)) {
            Files.write(path, content.getBytes(), StandardOpenOption.APPEND);
        } else {
            Files.write(path, content.getBytes(), StandardOpenOption.CREATE);
        }
    }
 
    public static void main(String[] args) {
    //     FilePrinter fp = new FilePrinter();
    //     try {
    //         fp.write(0.5, 20.0, 1);
    //         fp.write(0.5, 20.0, 1);
    //         fp.write(0.5, 23.0, 2);
    //         fp.write(0.5, 23.0, 1);
    //         fp.write(0.5, 23.0, 2);
    //     } catch (IOException e) {
    //         // TODO Auto-generated catch block
    //         e.printStackTrace();
    //     }
    // }
 
    }
}
