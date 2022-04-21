package se.oru.coordination.coordination_oru.MAS;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;    
 
// Main class
public class FilePrinter {
    protected String separator = ",";
    private String path = "/home/parallels/Projects/coordination_oru/testResults/testRun";
    protected ArrayList<Integer> robots = new ArrayList<Integer>();

    public FilePrinter() {
        new File(this.path).mkdirs();
    }

    protected void getDataAndTime() {
        DateTimeFormatter dtf = DateTimeFormatter.ofPattern("MM/dd_HH:mm:ss");  
        LocalDateTime now = LocalDateTime.now();  
        System.out.println(dtf.format(now));  
    }

    /**
     * For writing storage state into file
     * @param time
     * @param ore
     * @throws IOException
     */
    protected void write(double time, double actualOre, double oreState, int robotID) {
        Path path = Path.of(this.path + "/OreState" + robotID + ".csv");
        String content = time + this.separator + actualOre + this.separator + oreState + "\n";
        this.writeToFile(path, content);
    }

    /**
     * Function that logs messages with times
     * @param time
     */
    protected void addMessageCounter(Double time, int messageAmount) {
        Path path = Path.of(this.path + "/Messages" + ".csv");
        String content = time + this.separator + messageAmount + "\n";
        this.writeToFile(path, content);
    } 

    /**
     * 
     * @param time
     * @param waitTime
     */
    protected void addWaitingTimeMeasurment(Double time, double waitTime, int robotID) {
        Path path = Path.of(this.path + "/WaitingTimes" + robotID + ".csv");
        String content = time + this.separator + waitTime + "\n";
        this.writeToFile(path, content);
    } 

    /**
     * Writes to file with provided path and content
     * @param path
     * @param content
     */
    private void writeToFile(Path path, String content) {
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
        FilePrinter fp = new FilePrinter(); 
        fp.addMessageCounter(0.0,5);
        fp.write(0.2, 5, 4, 2);
        fp.addWaitingTimeMeasurment(10.0, 23.0, 2);
 
    }
}
