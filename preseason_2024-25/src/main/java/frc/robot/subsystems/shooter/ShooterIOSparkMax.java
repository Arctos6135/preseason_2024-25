package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.CANBusConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SwerveConstants;

public class ShooterIOSparkMax extends ShooterIO {
    private final CANSparkMax leader = new CANSparkMax(CANBusConstants.SHOOTER_LEADER, MotorType.kBrushless);
    private final CANSparkMax follower = new CANSparkMax(CANBusConstants.SHOOTER_FOLLOWER, MotorType.kBrushless);

    public ShooterIOSparkMax(){
        leader.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        follower.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);

        leader.setInverted(false);
        follower.setInverted(false);

        follower.follow(leader, true);

        leader.setIdleMode(IdleMode.kBrake);
        follower.setIdleMode(IdleMode.kBrake);
    }

    public void setVoltages(double voltage){
        leader.setVoltage(voltage);
    }
}
