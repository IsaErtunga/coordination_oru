package se.oru.coordination.coordination_oru.MAS;

import java.io.*;
import java.util.HashMap;
import java.util.Scanner;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption; 

public class TestFramework {
    public static int EXPERIMENTS = 14;
    public static int EMPERIMENT_TIME = 5 * 60;
    public static int CURRENT_EXPERIMENT;
    private static String experimentValuesPath = "/home/parallels/Projects/coordination_oru/experimentValues";

    /**
     * Writes to file with provided path and content
     * @param path
     * @param content
     */
    private static void writeToFile(String path, String content) {
        Path realPath = Path.of(experimentValuesPath + path);
        try {
            Files.write(realPath, content.getBytes(), StandardOpenOption.TRUNCATE_EXISTING);
        } catch (IOException e) {
                e.printStackTrace();
        }
    }

    public static void runTest(String content) throws IOException {
        System.out.println("-----------------------------------------------");
        System.out.println("Starting experiment ...");

        // Create new file for values in experiment
        // File(experimentValuesPath).mkdirs();
        
        //String content = readCSV();
        String runCommand = "./gradlew run -Pdemo=testMAS.NewMapTesting";

        Process proc;
        proc = Runtime.getRuntime().exec(runCommand);
        long procID = proc.pid();
        BufferedReader reader = new BufferedReader(new InputStreamReader(proc.getInputStream()));
        long startTime = System.currentTimeMillis();
        // long endTime = startTime + (EMPERIMENT_TIME * 1000);

        writeToFile("/values.csv", procID +","+ content);

        while ( proc.isAlive() ){
            String line = "";
            line = reader.readLine();
            if (line != null) {
                System.out.print(line + "\n");
            }     
        }
        System.out.println("Done with experiment, took: " + (System.currentTimeMillis() - startTime)/1000 + " seconds");
        proc.destroy();
    }

    public static void main(String[] args) throws FileNotFoundException  {
        String experimentDataPath = "/home/parallels/Projects/coordination_oru/experimentData/OriginalSystemExperiments.csv";
        Scanner sc = new Scanner(new File(experimentDataPath));
        sc.useDelimiter(",");
        for (int i = 0; i < EXPERIMENTS; i++) {
            int numCols = 8;
            String result = "";

            for (int col = 0; col < numCols; col++) {
                if (sc.hasNext()) {
                    String value = sc.next();
                    result = result + value + ",";
                }
            }
            result = result.replaceAll("\\r|\\n", "");
            try {
                runTest(result);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        sc.close();
    }
}