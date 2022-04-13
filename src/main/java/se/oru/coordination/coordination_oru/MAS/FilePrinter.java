package se.oru.coordination.coordination_oru.MAS;


// Java Program to Write Into a File
// using writeString() Method
 
// Importing required classes
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
 
// Main class
public class FilePrinter {
    protected String fileName;
    protected Path path;
    protected String separator = ",";

    public FilePrinter() {
        this.fileName = "hej.csv";
        this.path = Path.of("/home/parallels/" + this.fileName);
    }

    public FilePrinter(String path, String fileName) {
        this.path = Path.of(path + this.fileName);
    }

    /**
     * For writing storage state into file
     * @param time
     * @param ore
     * @throws IOException
     */
    protected void write(double time, double ore) throws IOException {
        String content = time + this.separator + ore + "\n";
        if (Files.exists(this.path)) {
            Files.write(this.path, content.getBytes(), StandardOpenOption.APPEND);
        } else {
            Files.write(this.path, content.getBytes(), StandardOpenOption.CREATE);
        }
    }
 
    public static void main(String[] args) {
        FilePrinter fp = new FilePrinter();
        try {
            fp.write(0.5, 20.0);
            fp.write(0.5, 30.0);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
