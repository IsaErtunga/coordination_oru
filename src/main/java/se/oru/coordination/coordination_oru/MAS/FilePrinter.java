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
    protected boolean isActive;
    private String path = "/home/parallels/Projects/coordination_oru/testResults/testRun";
    protected ArrayList<Integer> robots = new ArrayList<Integer>();

    public FilePrinter(boolean isActive) {
        this.isActive = isActive;
        if (isActive) {
            new File(this.path).mkdirs();
        }
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
    protected void write(double time, double ore, int robotID) {
        if (isActive) {
            Path path = Path.of(this.path + "/OreState" + robotID + ".csv");
            String content = time + this.separator + ore + "\n";
            this.writeToFile(path, content);
        }
    }

    /**
     * Function that logs messages with times
     * @param time
     */
    protected void addMessageCounter(Double time, String messageType) {
        if (isActive) {
            Path path = Path.of(this.path + "/Messages" + ".csv");
            String content = time + this.separator + messageType + "\n";
            this.writeToFile(path, content);
        }
    } 

    /**
     * 
     * @param time
     * @param waitTime
     */
    protected void addWaitingTimeMeasurment(String type, double waitTime, int robotID) {
        if (isActive) {
            Path path = Path.of(this.path + "/WaitingTimes" + robotID + ".csv");
            String content = type + this.separator + waitTime + "\n";
            this.writeToFile(path, content);
        }
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
        // FilePrinter fp = new FilePrinter(); 

    }
}