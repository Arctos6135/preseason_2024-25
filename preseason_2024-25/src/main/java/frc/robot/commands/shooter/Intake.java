package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class Intake extends Command{

    Shooter shooter;

    public Intake(Shooter shooter){
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        shooter.setVoltages(-4);
    }

    @Override
    public void end(boolean interrupted){
        shooter.setVoltages(0);
    }
    
}
