package frc.robot.commands.characterization;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TurningVelocityRoutine extends Command {
    private final LoggedMechanismGroup mechanism;
    private final double maxDistance;
    private final double rampRate;
    private double voltage = 0;
    private double lastTimestamp;

    public TurningVelocityRoutine(LoggedMechanismGroup mechanism, double maxDistance, double rampRate, Subsystem... requirements) {
        this.mechanism = mechanism;
        this.maxDistance = maxDistance;
        this.rampRate = rampRate;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Running", true);
        lastTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        Logger.recordOutput("Running at voltage", voltage);
        System.out.printf("Running at voltage %f\n", voltage);
        double timestamp = Timer.getFPGATimestamp();
        voltage += rampRate * (timestamp - lastTimestamp);
        lastTimestamp = timestamp;

        mechanism.setVoltage(voltage);
    }

    @Override
    public void end(boolean interupted) {
        Logger.recordOutput("Running", false);

        mechanism.setVoltage(0);
    }
}
