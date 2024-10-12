package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private ShooterIO io;

    public Shooter(ShooterIO io){
        this.io = io;
    }
    
    public void setVoltages(double voltage){
        io.setVoltages(voltage);
    }
}
