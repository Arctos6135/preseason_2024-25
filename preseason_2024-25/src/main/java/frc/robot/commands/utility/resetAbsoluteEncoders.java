package frc.robot.commands.utility;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class resetAbsoluteEncoders extends Command {
    private final Swerve swerve;

    public resetAbsoluteEncoders(Swerve swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.resetAbsoluteEncoders();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}