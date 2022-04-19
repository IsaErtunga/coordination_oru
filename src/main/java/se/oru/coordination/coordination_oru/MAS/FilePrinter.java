package se.oru.coordination.coordination_oru.MAS;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
 
// Main class
public class FilePrinter {
    protected String separator = ",";
    private String path = "/home/parallels/";
    private String folderName = "testRun";
    protected ArrayList<Integer> robots = new ArrayList<Integer>();

    public FilePrinter() {
        new File("/home/parallels/"+this.folderName).mkdirs();
    }

    /**
     * For writing storage state into file
     * @param time
     * @param ore
     * @throws IOException
     */
    protected void write(double time, double actualOre, double oreState, int robotID) {
        Path path = Path.of(this.path + this.folderName + "/OreState" + robotID + ".csv");
        String content = time + this.separator + actualOre + this.separator + oreState + "\n";
        try {
            if (Files.exists(path)) {
                Files.write(path, content.getBytes(), StandardOpenOption.APPEND);
            } else {
                Files.write(path, content.getBytes(), StandardOpenOption.CREATE);
            }
        } catch (IOException e) {
                e.printStackTrace();
        }
    }

    /**
     * Function that logs messages with times
     * @param time
     */
    protected void addMessageCounter(Double time) {
        Path path = Path.of(this.path + this.folderName + "/Messages" + ".csv");
        String content = time + this.separator + "1" + "\n";
        try {
            if (Files.exists(path)) {
                Files.write(path, content.getBytes(), StandardOpenOption.APPEND);
            } else {
                Files.write(path, content.getBytes(), StandardOpenOption.CREATE);
            }
        } catch (IOException e) {
                e.printStackTrace();
        }
    } 

    /**
     * 
     * @param time
     * @param waitTime
     */
    protected void addWaitingTimeMeasurment(Double time, double waitTime, int robotID) {
        Path path = Path.of(this.path + this.folderName + "/WaitingTimes" + robotID + ".csv");
        String content = time + this.separator + waitTime + "\n";
        try {
            if (Files.exists(path)) {
                Files.write(path, content.getBytes(), StandardOpenOption.APPEND);
            } else {
                Files.write(path, content.getBytes(), StandardOpenOption.CREATE);
            }
        } catch (IOException e) {
                e.printStackTrace();
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
