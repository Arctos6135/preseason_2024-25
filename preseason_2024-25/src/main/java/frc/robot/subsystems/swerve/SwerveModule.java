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
        // Sets up the motors.
        this.drivingMotor = new CANSparkMax(CANBusConstants.DRIVE_IDS.get(moduleIdentifier), MotorType.kBrushless);
        this.turningMotor = new CANSparkMax(CANBusConstants.TURN_IDS.get(moduleIdentifier), MotorType.kBrushless);

        // Creates the analog (absolute) encoder.
        absoluteTurningEncoder = new AnalogEncoder(SwerveConstants.ENCODER_PORTS.get(moduleIdentifier));

        // Configures the motors to the proper settings.
        configDriveMotor();
        configTurningMotor();

        // Creates the encoders.
        this.drivingEncoder = drivingMotor.getEncoder();
        this.turningEncoder = turningMotor.getEncoder();

        // Creates SparkPIDControllers.
        this.drivingPIDController = drivingMotor.getPIDController();
        this.turningPIDController = turningMotor.getPIDController();

        // Links the controllers to the encoders.
        drivingPIDController.setFeedbackDevice(drivingEncoder);
        turningPIDController.setFeedbackDevice(turningEncoder);

        this.chassisAngularOffset = SwerveConstants.ANGULAR_OFFSETS.get(moduleIdentifier);

        // Enables PID wrapping to take more efficient routes when turning.
        turningPIDController.setPositionPIDWrappingEnabled(true);
        turningPIDController.setPositionPIDWrappingMinInput(0);
        turningPIDController.setPositionPIDWrappingMaxInput(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);

        // Disabled for driving because it doesn't wrap around.
        drivingPIDController.setPositionPIDWrappingEnabled(false);

        // Sets conversion factors so the encoder tracks in meters instead of rotations.
        drivingEncoder.setPositionConversionFactor(SwerveConstants.DRIVING_ENCODER_POSITION_FACTOR);
        drivingEncoder.setVelocityConversionFactor(SwerveConstants.DRIVING_ENCODER_VELOCITY_FACTOR);
        turningEncoder.setPositionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
        turningEncoder.setVelocityConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);

        // Sets a conversion factor on the analog controller.
        absoluteTurningEncoder.setDistancePerRotation(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);

        // Zeroes the position.
        drivingEncoder.setPosition(0.0);

        // Resets the turning position to match the absolute position.
        resetToAbsolute();

        // Set PID values.
        turningPIDController.setP(SwerveConstants.TURNING_PID[moduleIdentifier][0]);
        turningPIDController.setI(SwerveConstants.TURNING_PID[moduleIdentifier][1]);
        turningPIDController.setD(SwerveConstants.TURNING_PID[moduleIdentifier][2]);
        drivingPIDController.setP(SwerveConstants.DRIVING_PID[moduleIdentifier][0]);
        drivingPIDController.setI(SwerveConstants.DRIVING_PID[moduleIdentifier][1]);
        drivingPIDController.setD(SwerveConstants.DRIVING_PID[moduleIdentifier][2]);
    }

    /**
     * Resets the turning encoders to the angle given by the analog encoders.
     */
    private void resetToAbsolute() {
        // double absolutePosition = Math.abs(absoluteTurningEncoder.get() - absoluteTurningEncoder.getPositionOffset());
        turningEncoder.setPosition(0);
    }

    private void configTurningMotor() {
        // Returns the motor to its factory settings.
        turningMotor.restoreFactoryDefaults();
        // Sets a limit on the current draw of the motor.
        turningMotor.setSmartCurrentLimit(SwerveConstants.TURNING_CURRENT_LIMIT);
        // Makes it so the motor spins clockwise.
        turningMotor.setInverted(true);
        // When doing nothing, the motor will brake.
        turningMotor.setIdleMode(IdleMode.kBrake);

        // Saves the changes so they retain throughout power cycles.
        turningMotor.burnFlash();
    }

    private void configDriveMotor() {
        drivingMotor.restoreFactoryDefaults();
        drivingMotor.setSmartCurrentLimit(SwerveConstants.DRIVING_CURRENT_LIMIT);
        drivingMotor.setInverted(false);
        drivingMotor.setIdleMode(IdleMode.kBrake);
        drivingMotor.burnFlash();
    }

    /**
     * Gets the distance in meters.
     */
    public double getDistance() {
        return drivingEncoder.getPosition();
    }

    /**
     * Gets the angle of the swerve module.
     * @return module angle
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(turningEncoder.getPosition() % (2 * Math.PI)); // keeps in least terms.
    }

    /**
     * Gets the position of the module.
     * @return module position
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    /**
     * Gets the state of the module.
     * @return module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDistance(), getAngle());
    }

    /**
     * Sets the desired state of the module.
     * @param desiredState
     */
    public void setState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        // Accounts for the angular offset of the swerve module.
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState,
            new Rotation2d(turningEncoder.getPosition())
        );

        // Sets the setpoint for the PID controllers to follow.
        drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity); // m/s
        turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition); // radians
    }
}