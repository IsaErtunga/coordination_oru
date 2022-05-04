package se.oru.coordination.coordination_oru.MAS;

import java.io.*;
import java.util.HashMap;
import java.util.Scanner;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path; 

public class TestFramework {
    public static int experiments = 1;
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
        
            if (counter != 0) {
                result = result + value + ",";
            }

            counter++;
        }
        sc.close();
        return result;

    }

    public static void main(String[] args) throws IOException, InterruptedException {
        for (int i = 0; i < experiments; i++) {
            // Create new file for values in experiment
            // File(experimentValuesPath).mkdirs();

            String content = readCSV();
            System.out.println(content);
            writeToFile("/values.csv", content);
            // String command = "cd Projects/";
            // String runCommand = "./gradlew run -Pdemo=testMAS.NewMapTesting";
            // Process proc = Runtime.getRuntime().exec(runCommand);

            // BufferedReader reader =  
            // new BufferedReader(new InputStreamReader(proc.getInputStream()));

            // String line = "";
            // while((line = reader.readLine()) != null) {
            //     System.out.print(line + "\n");
            // }

            // proc.waitFor();   
        }
    }
}
