package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.SwerveConstants;
import frc.robot.util.SimMotor;

public class SimModule {
    private final SimMotor drivingMotor;
    private final DCMotorSim turningMotor;

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
    public SimModule(int moduleIdentifier) {
        // Sets up the motors.
        this.drivingMotor = new SimMotor(new double[][]{{0.06475, 0.05327}, {0.158, 0.7688}}, new double[][]{{0.03167}, {-0.002221}}, new double[]{11.21, 4.132}, new double[]{0.0});
        this.turningMotor = new DCMotorSim(SwerveConstants.TURNING_MOTOR_LINEAR_SYSTEMS.get(moduleIdentifier), DCMotor.getNEO(1), (150.0 / 7.0));

        this.moduleIdentifier = modules[moduleIdentifier];

        turningMotor.setState(0, 0);

        // Creates PIDController with the module's unique gains.
        this.drivingPIDController = new PIDController(SwerveConstants.DRIVING_PID[moduleIdentifier][0], SwerveConstants.DRIVING_PID[moduleIdentifier][1], SwerveConstants.DRIVING_PID[moduleIdentifier][2]);
        this.turningPIDController = new PIDController(SwerveConstants.TURNING_PID[moduleIdentifier][0], SwerveConstants.TURNING_PID[moduleIdentifier][1], SwerveConstants.TURNING_PID[moduleIdentifier][2]);

        // Creates feedforward controllers for the driving motor.
        this.drivingFeedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVING_FEEDFORWARDS[moduleIdentifier][0], SwerveConstants.DRIVING_FEEDFORWARDS[moduleIdentifier][1], SwerveConstants.DRIVING_FEEDFORWARDS[moduleIdentifier][2]);

        // Enables PID wrapping to take more efficient routes when turning.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Disabled for driving because it doesn't wrap around.
        drivingPIDController.disableContinuousInput();

        // Gets an initial time value for use in acceleration calculation.
        lastTime = Timer.getFPGATimestamp();

        // Sets the last velocity of module motors to zero.
        lastTargetDrivingVelocity = 0.0;
    }

    /**
     * Resets the turning encoders to the angle given by the analog encoders.
     */
    private void resetToAbsolute() {
        turningMotor.setState(0.0, 0.0);
    }

    /**
     * Gets the position in meters.
     * @return position of the module in meters
     */
    public double getPosition() {
        return drivingMotor.getPosition();
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
        return drivingMotor.getVelocity();
    }

    /**
     * Gets the angle of the swerve module.
     * @return module angle (radians)
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(turningMotor.getAngularPositionRotations() * SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
    }

    /**
     * Gets the velocity of the turning motor in rad/s.
     * @return angular velocity (rad/s)
     */
    public double getTurningVelocity() {
        return turningMotor.getAngularVelocityRadPerSec() / (150.0 / 7);
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
        return new SwerveModuleState(getDrivingVelocity(), getAngle());
    }

    /**
     * Gets the current of the module's driving motor.
     * @return driving motor current
     */
    public double getDrivingCurrent() {
        return 0.0;
    }

    /**
     * Gets the current of the module's turning motor.
     * @return turning motor current
     */
    public double getTurningCurrent() {
        return turningMotor.getCurrentDrawAmps();
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
        return 0.0;
    }

    /**
     * Gets the acceleration of the turning motor.
     * @return the acceleration of the turning motor.
     */
    public double getTurningAcceleration() {
        return turningAcceleration;
    }

    public void setDrivingVoltage(double voltage) {
        drivingMotor.setVoltage(voltage);
    }

    public void update() {
        drivingMotor.update(0.02);
        turningMotor.update(0.02);
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

        targetDrivingVelocity = optimizedDesiredState.speedMetersPerSecond;

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
        turningMotor.setInputVoltage(MathUtil.clamp(turningVoltage, -12, 12));

        // Updates the last values.
        lastTargetDrivingVelocity = targetDrivingVelocity;

        velocitySetpoint = optimizedDesiredState.speedMetersPerSecond;
        angleSetpoint = optimizedDesiredState.angle.getRadians();
    }
}