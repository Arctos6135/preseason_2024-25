package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.constants.OtherConstants;
import frc.robot.constants.SwerveConstants;

public class SwerveModule {
    private final TalonFX drivingMotor;
    private final CANSparkMax turningMotor;

    private final AnalogEncoder absoluteTurningEncoder;
    private final RelativeEncoder integratedTurningEncoder;

    private final SparkPIDController turningPIDController;

    private final TalonFXConfigurator drivingConfigurator;
    private TalonFXConfiguration drivingConfigs;

    private double chassisAngularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs an MK4i swerve module and configures the driving and turning motor, encoder, and PID controller.
     */
    public SwerveModule(int drivingCANId, int turningCANId, int encoderCANId, double chassisAngularOffset) {
        drivingMotor = new TalonFX(drivingCANId);
        turningMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);
        absoluteTurningEncoder = new AnalogEncoder(OtherConstants.GYRO_PORT);

        drivingConfigurator = drivingMotor.getConfigurator();
        
        // Creates a PID controller and the built-in encoder.
        integratedTurningEncoder = turningMotor.getEncoder();
        turningPIDController = turningMotor.getPIDController();
        turningPIDController.setFeedbackDevice(integratedTurningEncoder);
        

        configTurningEncoder();
        configTurningMotor();
        configDriveMotor();
    }

    private void resetToAbsolute() {
        double absolutePosition = Math.abs(absoluteTurningEncoder.get() - absoluteTurningEncoder.getPositionOffset());
        integratedTurningEncoder.setPosition(absolutePosition);
    }

    private void configTurningEncoder() {
        absoluteTurningEncoder.setDistancePerRotation(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
    }

    private void configTurningMotor() {
        turningMotor.restoreFactoryDefaults();
        turningMotor.setSmartCurrentLimit(SwerveConstants.TURNING_CURRENT_LIMIT);
        turningMotor.setInverted(true); // unsure if this should be inverted or not.
        turningMotor.setIdleMode(IdleMode.kBrake);

        turningPIDController.setPositionPIDWrappingEnabled(true);
        turningPIDController.setPositionPIDWrappingMinInput(0);
        turningPIDController.setPositionPIDWrappingMaxInput(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);

        integratedTurningEncoder.setPositionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
        integratedTurningEncoder.setVelocityConversionFactor(SwerveConstants.TURNING_VELOCITY_CONVERSION);
        resetToAbsolute();

        turningMotor.burnFlash();
    }

    private void configDriveMotor() {
        drivingConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        drivingConfigs.CurrentLimits.StatorCurrentLimit = 40;
        drivingConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        drivingConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        drivingConfigurator.apply(drivingConfigs);

        desiredState.angle = new Rotation2d(absoluteTurningEncoder.getDistance());
        drivingMotor.setPosition(0);
    }

    /**
     * Gets the distance in meters.
     */
    public double getDistance() {
        return drivingMotor.getPosition().getValueAsDouble() * SwerveConstants.DRIVING_ENCODER_POSITION_FACTOR;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(absoluteTurningEncoder.getDistance());
    }

    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(
            getDistance(),
            getAngle()
        );
    }

    public void setState(SwerveModuleState desiredState) {
        turningPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
        drivingMotor.set(desiredState.speedMetersPerSecond);
    }
}
