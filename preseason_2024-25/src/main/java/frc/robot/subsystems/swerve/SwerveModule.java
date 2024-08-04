package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.constants.CANBusConstants;
import frc.robot.constants.SwerveConstants;

public class SwerveModule {
    private final CANSparkMax drivingMotor;
    private final CANSparkMax turningMotor;

    private final AnalogEncoder absoluteTurningEncoder;

    private final RelativeEncoder drivingEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController drivingPIDController;
    private final PIDController turningPIDController;

    public double angleSetpoint;
    public double velocitySetpoint;

    private double chassisAngularOffset = 0;

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

        // Creates PIDController with the module's unique gains.
        this.drivingPIDController = new PIDController(SwerveConstants.DRIVING_PID[moduleIdentifier][0], SwerveConstants.DRIVING_PID[moduleIdentifier][1], SwerveConstants.DRIVING_PID[moduleIdentifier][2]);
        this.turningPIDController = new PIDController(SwerveConstants.TURNING_PID[moduleIdentifier][0], SwerveConstants.TURNING_PID[moduleIdentifier][1], SwerveConstants.TURNING_PID[moduleIdentifier][2]);

        // Sets conversion factors so the encoder tracks in meters instead of rotations.
        drivingEncoder.setPositionConversionFactor(SwerveConstants.DRIVING_ENCODER_POSITION_FACTOR);
        drivingEncoder.setVelocityConversionFactor(SwerveConstants.DRIVING_ENCODER_VELOCITY_FACTOR);
        turningEncoder.setPositionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
        turningEncoder.setVelocityConversionFactor(SwerveConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        this.chassisAngularOffset = SwerveConstants.ANGULAR_OFFSETS.get(moduleIdentifier);

        // Enables PID wrapping to take more efficient routes when turning.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Disabled for driving because it doesn't wrap around.
        drivingPIDController.disableContinuousInput();

        // Sets a conversion factor on the analog controller.
        absoluteTurningEncoder.setDistancePerRotation(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);

        // Zeroes the position.
        drivingEncoder.setPosition(0.0);

        // Resets the turning position to match the absolute position.
        resetToAbsolute();
    }

    /**
     * Resets the turning encoders to the angle given by the analog encoders.
     */
    private void resetToAbsolute() {
        Rotation2d absolutePosition = Rotation2d.fromRotations(absoluteTurningEncoder.getAbsolutePosition() - absoluteTurningEncoder.getPositionOffset());
        turningEncoder.setPosition(absolutePosition.getRadians());
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
     * Gets the position in meters.
     */
    public double getPosition() {
        return drivingEncoder.getPosition();
    }

    public void setDrivingPID(double P, double I, double D) {
        drivingPIDController.setP(P);
        drivingPIDController.setI(I);
        drivingPIDController.setD(D);
    }

    public void setTurningPID(double P, double I, double D) {
        turningPIDController.setP(P);
        turningPIDController.setI(I);
        turningPIDController.setD(D);
    }

    /**
     * Gets the velocity of the driving motor in meters per second.
     * @return driving velocity
     */
    public double getDrivingVelocity() {
        return drivingEncoder.getVelocity();
    }

    /**
     * Gets the angle of the swerve module.
     * @return module angle
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turningEncoder.getPosition());
    }

    /**
     * Gets the velocity of the turning motor in rad/s.
     * @return angular velocity
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /**
     * Gets the position of the module.
     * @return module position
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    /**
     * Gets the state of the module.
     * @return module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getPosition(), getAngle());
    }

    public double getDrivingCurrent() {
        return drivingMotor.getOutputCurrent();
    }

    public double getTurningCurrent() {
        return drivingMotor.getOutputCurrent();
    }

    public double getDrivingVoltage() {
        return drivingMotor.getAppliedOutput();
    }

    public double getTurningVoltage() {
        return turningMotor.getAppliedOutput();
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
            getAngle()
        );

        // Sets the setpoint for the PID controllers to follow.
        drivingPIDController.setSetpoint(optimizedDesiredState.speedMetersPerSecond); // m/s
        turningPIDController.setSetpoint(optimizedDesiredState.angle.getRadians()); // radians

        // Calculates and sets the voltages of the motors.
        drivingMotor.setVoltage(MathUtil.clamp(drivingPIDController.calculate(drivingEncoder.getVelocity()), -12, 12));
        turningMotor.setVoltage(MathUtil.clamp(turningPIDController.calculate(getAngle().getRadians()), -12, 12));

        velocitySetpoint = optimizedDesiredState.speedMetersPerSecond;
        angleSetpoint = optimizedDesiredState.angle.getDegrees();
    }
}