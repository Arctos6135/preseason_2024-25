package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class TeleopDrive extends Command {
    private Swerve drivetrain;
    private DoubleSupplier xSpeedSupplier;
    private DoubleSupplier ySpeedSupplier;
    private DoubleSupplier rotSpeedSupplier;

    public TeleopDrive(Swerve drivetrain, DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, DoubleSupplier rotSpeedSupplier) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Probably some of the PID stuff goes here
    }

    @Override
    public void execute() {
        // Very simple version of this but good enough for now

        double xSpeed = xSpeedSupplier.getAsDouble();
        double ySpeed = ySpeedSupplier.getAsDouble();
        double rotSpeed = rotSpeedSupplier.getAsDouble();

        // Update Swerve module states based on inputs
        drivetrain.drive(xSpeed, ySpeed, rotSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}