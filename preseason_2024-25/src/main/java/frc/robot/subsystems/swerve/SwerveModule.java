package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.CANBusConstants;
import frc.robot.constants.SwerveConstants;

public class SwerveModule {
    private final TalonFX drivingMotor;
    private final CANSparkMax turningMotor;

    private final AnalogEncoder absoluteTurningEncoder;

    private final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    private final TalonFXConfigurator driveMotorConfigurator;

    private final RelativeEncoder turningEncoder;

    private final PIDController drivingPIDController;
    private final PIDController turningPIDController;

    private final SimpleMotorFeedforward drivingFeedforward;

    public double drivingVoltage;
    public double turningVoltage;

    public double drivingVelocity;
    public double turningVelocity;

    public double targetDrivingVelocity;

    public double lastTargetDrivingVelocity;

    public double targetDrivingAcceleration;
    public double turningAcceleration;

    public double angleSetpoint;
    public double velocitySetpoint;

    public double time;
    public double lastTime;

    private double chassisAngularOffset = 0;

    private String moduleIdentifier;

    private static String[] modules = {"frontLeft", "frontRight", "backLeft", "backRight"};

    /**
     * Constructs an MK4i swerve module and configures the driving and turning motor, encoder, and PID controller.
     */
    public SwerveModule(int moduleIdentifier) {
        // Sets up the motors.
        this.drivingMotor = new TalonFX(CANBusConstants.DRIVE_IDS.get(moduleIdentifier));
        this.turningMotor = new CANSparkMax(CANBusConstants.TURN_IDS.get(moduleIdentifier), MotorType.kBrushless);

        driveMotorConfigurator = drivingMotor.getConfigurator();

        this.moduleIdentifier = modules[moduleIdentifier];

        // Creates the analog (absolute) encoder.
        absoluteTurningEncoder = new AnalogEncoder(SwerveConstants.ENCODER_PORTS.get(moduleIdentifier));

        // Adds a position offset to the absolute encoder to account for initial encoder placement.
        absoluteTurningEncoder.setPositionOffset(SwerveConstants.POSITION_OFFSETS[moduleIdentifier]);

        // Configures the motors to the proper settings.
        configDriveMotor();
        configTurningMotor();

        // Creates the encoders.
        this.turningEncoder = turningMotor.getEncoder();

        // Creates PIDController with the module's unique gains.
        this.drivingPIDController = new PIDController(SwerveConstants.DRIVING_PID[moduleIdentifier][0], SwerveConstants.DRIVING_PID[moduleIdentifier][1], SwerveConstants.DRIVING_PID[moduleIdentifier][2]);
        this.turningPIDController = new PIDController(SwerveConstants.TURNING_PID[moduleIdentifier][0], SwerveConstants.TURNING_PID[moduleIdentifier][1], SwerveConstants.TURNING_PID[moduleIdentifier][2]);

        // Creates feedforward controllers for the driving motor.
        this.drivingFeedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVING_FEEDFORWARDS[moduleIdentifier][0], SwerveConstants.DRIVING_FEEDFORWARDS[moduleIdentifier][1], SwerveConstants.DRIVING_FEEDFORWARDS[moduleIdentifier][2]);

        // Sets conversion factors so the encoder tracks in meters instead of rotations.
        turningEncoder.setPositionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
        turningEncoder.setVelocityConversionFactor(SwerveConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        // Enables PID wrapping to take more efficient routes when turning.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Disabled for driving because it doesn't wrap around.
        drivingPIDController.disableContinuousInput();

        // Sets a conversion factor on the analog controller.
        absoluteTurningEncoder.setDistancePerRotation(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);

        // Resets the turning position to match the absolute position.
        resetToAbsolute();

        // Gets an initial time value for use in acceleration calculation.
        lastTime = Timer.getFPGATimestamp();

        // Sets the last velocity of module motors to zero.
        lastTargetDrivingVelocity = 0.0;
    }

    /**
     * Resets the turning encoders to the angle given by the analog encoders.
     */
    private void resetToAbsolute() {
        Rotation2d absolutePosition = Rotation2d.fromRotations((absoluteTurningEncoder.getAbsolutePosition() + absoluteTurningEncoder.getPositionOffset()));
        turningEncoder.setPosition(absolutePosition.getRadians());
    }

    /**
     * Configures the turning motor.
     */
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
        // sturningMotor.burnFlash();
    }

    /**
     * Configures the driving motor.
     */
    private void configDriveMotor() {
        // Enables and sets current limits.
        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.DRIVING_CURRENT_LIMIT;
        
        // Sets the neutral mode to brake mode.
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Sets the driving encoder position factor.
        driveMotorConfig.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVING_ENCODER_POSITION_FACTOR;
        
        // Sets the motor to be clockwise positive.
        driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        driveMotorConfigurator.apply(driveMotorConfig);

        // drivingMotor.burnFlash();
    }

    /**
     * Gets the position in meters.
     * @return position of the module in meters
     */
    public double getPosition() {
        return drivingMotor.getPosition().getValueAsDouble();
    }

    /**
     * Sets the gains of the driving PID controller.
     * @param P proportional gain
     * @param I integral gain
     * @param D derivative gain
     */
    public void setDrivingPID(double P, double I, double D) {
        drivingPIDController.setP(P);
        drivingPIDController.setI(I);
        drivingPIDController.setD(D);
    }

    /**
     * Sets the gains of the turning PID controller.
     * @param P proportional gain
     * @param I integral gain
     * @param D derivative gain
     */
    public void setTurningPID(double P, double I, double D) {
        turningPIDController.setP(P);
        turningPIDController.setI(I);
        turningPIDController.setD(D);
    }

    /**
     * Gets the velocity of the driving motor in meters per second.
     * @return driving velocity (m/s)
     */
    public double getDrivingVelocity() {
        return drivingMotor.get();
    }

    /**
     * Gets the angle of the swerve module.
     * @return module angle (radians)
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(turningEncoder.getPosition());
    }

    /**
     * Gets the angle reported by the absolute magnetic encoders.
     * @return absolute encoder angle (rotations)
     */
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(absoluteTurningEncoder.getAbsolutePosition());
    }

    /**
     * Gets the velocity of the turning motor in rad/s.
     * @return angular velocity (rad/s)
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

    /**
     * Gets the current of the module's driving motor.
     * @return driving motor current
     */
    public double getDrivingCurrent() {
        return drivingMotor.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Gets the current of the module's turning motor.
     * @return turning motor current
     */
    public double getTurningCurrent() {
        return turningMotor.getOutputCurrent();
    }

    /**
     * Gets the last voltage fed into the driving motor.
     * @return last voltage fed into the driving motor.
     */
    public double getDrivingVoltage() {
        return drivingVoltage;
    }

    /**
     * Gets the last voltage fed into the turning motor.   
     * @return last voltage fed into the turning motor
     */
    public double getTurningVoltage() {
        return turningVoltage;
    }

    /**
     * Gets the acceleration of the driving motor.
     * @return the acceleration of the driving motor.
     */
    public double getDrivingAcceleration() {
        return drivingMotor.getAcceleration().getValueAsDouble();
    }

    /**
     * Gets the acceleration of the turning motor.
     * @return the acceleration of the turning motor.
     */
    public double getTurningAcceleration() {
        return turningAcceleration;
    }

    public void resetAbsoluteEncoder() {
        absoluteTurningEncoder.reset();
    }

    public void setDrivingVoltage(double voltage) {
        drivingMotor.setVoltage(voltage);
    }

    /**
     * Sets the desired state of the module.
     * @param desiredState
     */
    public void setState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        targetDrivingVelocity = desiredState.speedMetersPerSecond;

        // Accounts for the angular offset of the swerve module.
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState,
            getAngle()
        );

        targetDrivingAcceleration = (targetDrivingVelocity - lastTargetDrivingVelocity) / 0.02;

        // Sets the setpoint for the PID controllers to follow.
        drivingPIDController.setSetpoint(optimizedDesiredState.speedMetersPerSecond); // m/s
        turningPIDController.setSetpoint(optimizedDesiredState.angle.getRadians()); // radians

        drivingVelocity = getDrivingVelocity();
        turningVelocity = getTurningVelocity();

        // Calculates the feedback voltage given by the PID controllers.
        drivingVoltage = drivingPIDController.calculate(drivingVelocity);
        turningVoltage = turningPIDController.calculate(getAngle().getRadians());

        drivingVoltage += drivingFeedforward.calculate(targetDrivingVelocity, targetDrivingAcceleration);

        // Sets the voltage of the motors clamped to reasonable values.
        drivingMotor.setVoltage(MathUtil.clamp(drivingVoltage, -12, 12));
        turningMotor.setVoltage(MathUtil.clamp(turningVoltage, -12, 12));

        // Updates the last values.
        lastTargetDrivingVelocity = targetDrivingVelocity;

        velocitySetpoint = optimizedDesiredState.speedMetersPerSecond;
        angleSetpoint = optimizedDesiredState.angle.getDegrees();
    }
}