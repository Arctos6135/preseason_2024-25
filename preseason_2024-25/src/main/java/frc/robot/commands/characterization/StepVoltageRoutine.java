package frc.robot.commands.characterization;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class StepVoltageRoutine extends Command {
    private static final double STEP_SIZE = 1.0; // Voltage step size
    private static final double MAX_VOLTAGE = 12.0; // Maximum voltage
    private static final double TIME_BETWEEN_STEPS = 0.02; // Time between steps in seconds
    private static final double MAX_DISTANCE = 5.0; // Maximum allowable distance in meters

    private final Swerve swerve; // Instance of the Swerve subsystem
    private double currentVoltage;
    private long lastStepTime;
    private Translation2d initialTranslation;

    public StepVoltageRoutine(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve); // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        // Initialize variables
        currentVoltage = 0.0;
        lastStepTime = System.currentTimeMillis();
        // Reset the distance traveled at the start of the command
        initialTranslation = swerve.getPose().getTranslation();
    }

    @Override
    public void execute() {
        // Get the current distance
        double distance = swerve.getPose().getTranslation().getDistance(initialTranslation);

        // Check if the distance has exceeded the maximum allowable distance
        if (distance >= MAX_DISTANCE) {
            System.out.println("Fail-safe triggered: Maximum distance exceeded.");
            cancel(); // Trigger the fail-safe
            return;
        }

        // Time control to ensure the step is applied at the correct interval
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastStepTime >= TIME_BETWEEN_STEPS * 1000) {
            // Set the voltage to all swerve modules
            swerve.setDrivingVoltage(currentVoltage);

            // Increase the voltage for the next step
            currentVoltage += STEP_SIZE;

            // Record the time of this step
            lastStepTime = currentTime;
        }
    }

    @Override
    public boolean isFinished() {
        // Finish if the voltage exceeds the maximum allowable voltage
        return currentVoltage > MAX_VOLTAGE;
    }

    @Override
    public void end(boolean interrupted) {
        // Reset the voltages to zero when the command ends
        swerve.setDrivingVoltage(0);
        if (interrupted) {
            System.out.println("Step voltage routine was interrupted.");
        } else {
            System.out.println("Step voltage routine completed successfully.");
        }
    }
}
