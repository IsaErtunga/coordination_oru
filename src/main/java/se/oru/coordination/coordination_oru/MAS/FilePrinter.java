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
    public HashMap<String, ArrayList<String>> savedValues = new HashMap<String, ArrayList<String>>();
    public ArrayList<String> experiments = new ArrayList<String>(); 

    private String path = "/home/parallels/Projects/coordination_oru/testResults/experiments";
    protected ArrayList<Integer> robots = new ArrayList<Integer>();

    public FilePrinter(boolean isActive) {
        this.isActive = isActive;
        if (isActive) {
            try {
                readValues();
                // this.path = this.path + this.EXPERIMENT_NR;
                // new File(this.path).mkdirs();
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
        }
    }

    public void readValues() throws FileNotFoundException {
        Scanner sc = new Scanner(new File("/home/parallels/Projects/coordination_oru/experimentValues/values.csv"));
        sc.useDelimiter(",");
        this.EXPERIMENT_NR = sc.next();  
        sc.close();
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
            String path = this.path + "/OreState" + robotID + ".csv";
            String content = time + this.separator + ore + "\n";
            //this.writeToFile(path, content);
            if (this.savedValues.containsKey(path)) {
                synchronized(this.savedValues) {
                    this.savedValues.get(path).add(content);
                }
            } else {
                synchronized(this.savedValues) {
                    this.savedValues.put(path, new ArrayList<String>());
                }
            }
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
            //this.writeToFile(path, content);
        }
    } 

    /**
     * 
     * @param time
     * @param waitTime
     */
    protected void addWaitingTimeMeasurment(String type, double waitTime, int robotID) {
        if (isActive) {
            String path = this.path + "/WaitingTimes" + robotID + ".csv";
            String content = path + this.separator +  type + this.separator + waitTime + "\n";
            //this.writeToFile(path, content);
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

    public void printSavedValues() {
        for (String path: this.savedValues.keySet()) {
            ArrayList<String> values = this.savedValues.get(path);
            System.out.println(path + " -> [" + values.toString() +"]");
        }
    }



    public void writeValueToFile() {
    
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
 
    public static void main(String[] args) {
        // FilePrinter fp = new FilePrinter(); 

    }
}