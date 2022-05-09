package se.oru.coordination.coordination_oru.MAS;

import java.io.*;
import java.util.HashMap;
import java.util.Scanner;
import java.nio.file.Files;
import java.nio.file.Path; 

public class TestFramework {
    public static int EXPERIMENTS = 2;
    public static int EMPERIMENT_TIME = 1 * 60;
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
            Files.write(realPath, content.getBytes());
        } catch (IOException e) {
                e.printStackTrace();
        }
    }

    public static String readCSV() throws FileNotFoundException {
        Scanner sc = new Scanner(new File("/home/parallels/Downloads/Testing Scenarios - Blad1.csv"));
        sc.useDelimiter(",");

        int numCols = 12;
        int counter = 0;
        String result = "";
 
        while (sc.hasNext()) {
            if (counter == numCols) break;
            String value = sc.next();

            result = result + value + ",";
            counter++;
        }
        sc.close();
        return result;

    }

    public static void runTest(String content) throws IOException {
        System.out.println("-----------------------------------------------");
        System.out.println("Starting experiment ...");

        // Create new file for values in experiment
        // File(experimentValuesPath).mkdirs();
        
        //String content = readCSV();
        writeToFile("/values.csv", content);
        String runCommand = "./gradlew run -Pdemo=testMAS.NewMapTesting";

        Process proc;
        proc = Runtime.getRuntime().exec(runCommand);
        BufferedReader reader = new BufferedReader(new InputStreamReader(proc.getInputStream()));

        long startTime = System.currentTimeMillis();
        long endTime = startTime + (EMPERIMENT_TIME * 1000);

        while (System.currentTimeMillis() < endTime) {
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
        Scanner sc = new Scanner(new File("/home/parallels/Downloads/Testing Scenarios - Blad1.csv"));
        sc.useDelimiter(",");
        for (int i = 0; i < EXPERIMENTS; i++) {
            int numCols = 12;
            int counter = 0;
            String result = "";

            for (int col = 0; col < numCols; col++) {
                if (sc.hasNext()) {
                    String value = sc.next();
                    result = result + value + ",";
                }
            }
            result = result.replace("\n", "");
            try {
                runTest(result);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        sc.close();
    }
}