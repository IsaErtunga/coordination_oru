package se.oru.coordination.coordination_oru.MAS;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;    

// Main class
public class FilePrinter {
    protected String separator = ",";
    protected boolean isActive;
    protected String EXPERIMENT_NR;
    protected long startTime;

    public ArrayList<String> loggedMessages;
    public ArrayList<String> experiments = new ArrayList<String>(); 
    
    private String path = "/home/parallels/Projects/coordination_oru/testResults/experiments.csv";
    protected ArrayList<Integer> robots = new ArrayList<Integer>();

    public FilePrinter(){};
    public FilePrinter(boolean isActive, ArrayList<String> loggedMessages, long startTime) {
        this.isActive = isActive;
        if (isActive) {
            try {
                readValues();
                this.loggedMessages = loggedMessages;
                this.startTime = startTime;
                // this.path = this.path + this.EXPERIMENT_NR;
                // new File(this.path).mkdirs();
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
        }
    }

    protected double getTime(){
        long diff = System.currentTimeMillis() - this.startTime;
        return (double)(diff)/1000.0;
    }

    public void readValues() throws FileNotFoundException {
        Scanner sc = new Scanner(new File("/home/parallels/Projects/coordination_oru/experimentValues/values.csv"));
        sc.useDelimiter(",");
        sc.next();  
        this.EXPERIMENT_NR = sc.next();  
        sc.close();
    }

    protected void getDataAndTime() {
        DateTimeFormatter dtf = DateTimeFormatter.ofPattern("MM/dd_HH:mm:ss");  
        LocalDateTime now = LocalDateTime.now();  
        System.out.println(dtf.format(now));  
    }

    protected void endExperiment(){
        String path = "/home/parallels/Projects/coordination_oru/experimentValues/experimentStatus.csv";

        String content = "1";

    }

    /**
     * For writing storage state into file
     * @param time
     * @param ore
     * @throws IOException
     */
    protected void logOreState(double time, double ore, int robotID) {
        if (isActive) {
            String content = this.EXPERIMENT_NR + this.separator + robotID + this.separator + "ORESTATE" +  this.separator + time + this.separator + ore + "\n";
            synchronized(this.experiments) {
                this.experiments.add(content);
            }
        }
    }

    /**
     * For writing storage state into file
     * @param time
     * @param ore
     * @throws IOException
     */
    protected void logCollectedOre(double ore) {
        if (isActive) {
            double timeStamp = this.getTime();
            String content = this.EXPERIMENT_NR + this.separator + "COLLECTED_ORE" +  this.separator + timeStamp + this.separator + ore + "\n";
            synchronized(this.experiments) {
                this.experiments.add(content);
            }
        }
    }

    /**
     * Function that logs messages with times
     * @param time
     */
    public void logMessages() {
        if (isActive) {
            ArrayList<String> loggedMessagesCopy;
            synchronized(this.loggedMessages) {
                loggedMessagesCopy = new ArrayList<String>(this.loggedMessages);
                this.loggedMessages.clear();
            }

            String content = "";
            for (String msg: loggedMessagesCopy) {
                content += this.EXPERIMENT_NR + this.separator + "MESSAGE" + this.separator + msg + "\n";
            }
            synchronized(this.experiments) {
                this.experiments.add(content);
            }
        }
    } 

        /**
     * 
     * @param time
     * @param waitTime
     */
    protected void addDistanceMeasurment(String type, double distance, int robotID) {
        if (isActive) {
            double timeStamp = this.getTime();
            String content = this.EXPERIMENT_NR + this.separator + robotID + this.separator + "DISTANCE" + this.separator + type + this.separator + timeStamp + this.separator + distance + "\n";
            synchronized(this.experiments) {
                this.experiments.add(content);
            }
        }
    } 

    /**
     * 
     * @param time
     * @param waitTime
     */
    protected void addWaitingTimeMeasurment(String type, double waitTime, int robotID) {
        if (isActive) {
            double timeStamp = this.getTime();
            String content = this.EXPERIMENT_NR + this.separator + robotID + this.separator + "TIME" + this.separator + type + this.separator + timeStamp + this.separator + waitTime + "\n";
            synchronized(this.experiments) {
                this.experiments.add(content);
            }
        }
    } 

    /**
     * Writes to file with provided path and content
     * @param path
     * @param content
     */
    private void writeToFile(String content) {
        Path path = Path.of(this.path);
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

    public void writeValueToFile() {
        if (isActive) {
            ArrayList<String> experimentsCopy;
            synchronized(this.experiments) {
                experimentsCopy = new ArrayList<String>(this.experiments);
                this.experiments.clear();
            }
            
            String testContent = "";
            for (String experiment: experimentsCopy) {
                testContent += experiment;
                //System.out.println("EXPERIMENT: ->" + experiment);
            }
            this.writeToFile(testContent);
        }
    }
 
    public static void main(String[] args) {
        FilePrinter fp = new FilePrinter();
        try {
            fp.readValues();
        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } 

    }
}