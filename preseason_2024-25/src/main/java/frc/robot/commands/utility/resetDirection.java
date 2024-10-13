package frc.robot.commands.utility;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class resetDirection extends Command {
    private final Swerve swerve;

    public resetDirection(Swerve swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.resetAbsoluteEncoders();
        swerve.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}