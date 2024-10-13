package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {
    
    public static Command shoot(Shooter shooter){
        return new InstantCommand(() -> shooter.setVoltages(6))
            .andThen(new WaitCommand(3))
            .andThen(new InstantCommand(() -> shooter.setVoltages(0)))
        ;
    }

}
