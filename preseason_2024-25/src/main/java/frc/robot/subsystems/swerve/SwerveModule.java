package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.CANBusConstants;

public class SwerveModule {
    private final CANSparkMax drivingMotor;
    private final CANSparkMax turningMotor;
    private final AnalogEncoder absoluteTurningEncoder;
    private final RelativeEncoder drivingEncoder;
    private final RelativeEncoder turningEncoder;
    private final SparkPIDController drivingPIDController;
    private final SparkPIDController turningPIDController;

    private double chassisAngularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs an MK4i swerve module and configures the driving and turning motor, encoder, and PID controller.
     */
    public SwerveModule(int moduleIdentifier) {
        this.drivingMotor = new CANSparkMax(CANBusConstants.DRIVE_IDS.get(moduleIdentifier), MotorType.kBrushless);
        this.turningMotor = new CANSparkMax(CANBusConstants.TURN_IDS.get(moduleIdentifier), MotorType.kBrushless);
        this.absoluteTurningEncoder = new AnalogEncoder(SwerveConstants.ENCODER_PORTS.get(moduleIdentifier));
        this.drivingEncoder = drivingMotor.getEncoder();
        this.turningEncoder = turningMotor.getEncoder();
        this.drivingPIDController = drivingMotor.getPIDController();
        this.turningPIDController = turningMotor.getPIDController();
        this.chassisAngularOffset = SwerveConstants.ANGULAR_OFFSETS.get(moduleIdentifier);

        turningPIDController.setP(SwerveConstants.TURNING_PID[moduleIdentifier][0]);
        turningPIDController.setI(SwerveConstants.TURNING_PID[moduleIdentifier][1]);
        turningPIDController.setD(SwerveConstants.TURNING_PID[moduleIdentifier][2]);

        drivingPIDController.setP(SwerveConstants.DRIVING_PID[moduleIdentifier][0]);
        drivingPIDController.setI(SwerveConstants.DRIVING_PID[moduleIdentifier][1]);
        drivingPIDController.setD(SwerveConstants.DRIVING_PID[moduleIdentifier][2]);

        configTurningEncoder();
        configTurningMotor();
        configDriveMotor();
    }

    private void resetToAbsolute() {
        double absolutePosition = Math.abs(absoluteTurningEncoder.get() - absoluteTurningEncoder.getPositionOffset());
        turningEncoder.setPosition(absolutePosition);
    }

    private void configTurningEncoder() {
        absoluteTurningEncoder.setDistancePerRotation(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
    }

    private void configTurningMotor() {
        turningMotor.restoreFactoryDefaults();
        turningMotor.setSmartCurrentLimit(SwerveConstants.TURNING_CURRENT_LIMIT);
        turningMotor.setInverted(true);
        turningMotor.setIdleMode(IdleMode.kBrake);

        turningPIDController.setPositionPIDWrappingEnabled(true);
        turningPIDController.setPositionPIDWrappingMinInput(0);
        turningPIDController.setPositionPIDWrappingMaxInput(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);

        turningEncoder.setPositionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
        turningEncoder.setVelocityConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
        resetToAbsolute();

        turningMotor.burnFlash();
    }

    private void configDriveMotor() {
        drivingMotor.restoreFactoryDefaults();
        drivingMotor.setSmartCurrentLimit(SwerveConstants.DRIVING_CURRENT_LIMIT);
        drivingMotor.setInverted(false);
        drivingMotor.setIdleMode(IdleMode.kBrake);

        drivingPIDController.setPositionPIDWrappingEnabled(false);

        drivingEncoder.setPositionConversionFactor(SwerveConstants.DRIVING_ENCODER_POSITION_FACTOR);
        drivingEncoder.setVelocityConversionFactor(SwerveConstants.DRIVING_ENCODER_VELOCITY_FACTOR);
        drivingEncoder.setPosition(0.0);

        drivingMotor.burnFlash();
    }

    /**
     * Gets the distance in meters.
     */
    public double getDistance() {
        return drivingEncoder.getPosition();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(turningEncoder.getPosition() % (2 * Math.PI));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDistance(), getAngle());
    }

    public void setState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState,
            new Rotation2d(turningEncoder.getPosition())
        );

        drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);
    }
}