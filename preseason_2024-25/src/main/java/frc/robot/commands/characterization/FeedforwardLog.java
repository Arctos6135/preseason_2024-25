package frc.robot.commands.characterization;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;

public class FeedforwardLog {
    private ArrayList<Double> voltages = new ArrayList<>();
    private ArrayList<Double> velocities = new ArrayList<>();
    private ArrayList<Double> accelerations = new ArrayList();

    public void accept(double voltage, double velocity, double acceleration) {
        voltages.add(voltage);
        velocities.add(velocity);
        accelerations.add(acceleration);
    }

    public void logCSV(String name) {
        try {
            File outputFile;
            outputFile = new File("/home/lvuser/" + name +"Feeforward.csv");
            System.out.println("Writing to file " + outputFile);
            outputFile.createNewFile();
            BufferedWriter out = new BufferedWriter(new FileWriter(outputFile));
            out.write("voltage,velocity,acceleration");
            out.newLine();
            for (int i = 0; i < voltages.size(); i++) {
                out.write(String.format("%f,%f,%f", voltages.get(i), velocities.get(i), accelerations.get(i)));
                out.newLine();
            }
            out.close();
            System.out.println("Wrote to file " + outputFile);
        } catch(Exception e) {
            DriverStation.reportError("Failed to create file to writefeedforward data", e.getStackTrace());
            System.out.println(e.getMessage());
        }
    }
}
