package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private ShooterIO io;

    private double targetVelocity = 0;
    private double lastTargetVelocity;

    public boolean velocityMode = false;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.FEEDFORWARD_VALUES[1], ShooterConstants.FEEDFORWARD_VALUES[2]);

    public Shooter(ShooterIO io){
        this.io = io;
    }
    
    public void setVoltages(double voltage){
        io.setVoltages(voltage);
    }

    public void setVelocity(double velocity) {
        targetVelocity = velocity;

    }

    @Override
    public void periodic() {
        if (velocityMode) {
            setVoltages(feedforward.calculate(lastTargetVelocity, targetVelocity, 0.02));
        }
    }
}
