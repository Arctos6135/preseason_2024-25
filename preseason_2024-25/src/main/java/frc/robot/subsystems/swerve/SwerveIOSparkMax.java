package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class SwerveIOSparkMax extends SwerveIO {
    // Gyro.
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    // Creates the swerve modules.
    private final SwerveModule frontLeft = new SwerveModule(0);
    private final SwerveModule frontRight = new SwerveModule(1);
    private final SwerveModule backLeft = new SwerveModule(2);
    private final SwerveModule backRight = new SwerveModule(3);

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
        frontLeft.getModulePosition(),
        frontRight.getModulePosition(),
        backLeft.getModulePosition(),
        backRight.getModulePosition()
        };
    }

    @Override
    public void setStates(SwerveModuleState[] moduleStates) {
        frontLeft.setState(moduleStates[0]);
        frontRight.setState(moduleStates[1]);
        backLeft.setState(moduleStates[2]);
        backRight.setState(moduleStates[3]);
    }

    @Override
    public void resetAbsoluteEncoders() {
        frontLeft.resetAbsoluteEncoder();
        frontRight.resetAbsoluteEncoder();
        backLeft.resetAbsoluteEncoder();
        backRight.resetAbsoluteEncoder();
    }

    @Override
    public void setDrivingVoltages(double voltage) {
        frontLeft.setDrivingVoltage(voltage);
        frontRight.setDrivingVoltage(voltage);
        backLeft.setDrivingVoltage(voltage);
        backRight.setDrivingVoltage(voltage);
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        // Front Left
        inputs.frontLeftPosition = frontLeft.getPosition();
        inputs.frontLeftAngle = frontLeft.getAngle().getDegrees();
        inputs.frontLeftAbsoluteAngle = frontLeft.getAbsoluteAngle().getRotations();
        inputs.frontLeftDrivingVelocity = frontLeft.getDrivingVelocity();
        inputs.frontLeftTurningVelocity = frontLeft.getTurningVelocity();
        inputs.frontLeftDrivingAcceleration = frontLeft.getDrivingAcceleration();
        inputs.frontLeftTurningAcceleration = frontLeft.getTurningAcceleration();
        inputs.frontLeftDrivingCurrent = frontLeft.getDrivingCurrent();
        inputs.frontLeftTurningCurrent = frontLeft.getTurningCurrent();
        inputs.frontLeftDrivingVoltage = frontLeft.getDrivingVoltage();
        inputs.frontLeftTurningVoltage = frontLeft.getTurningVoltage();
        inputs.frontLeftTargetVelocity = frontLeft.velocitySetpoint;
        inputs.frontLeftTargetAngle = frontLeft.angleSetpoint;

        // Front Right
        inputs.frontRightPosition = frontRight.getPosition();
        inputs.frontRightAngle = frontRight.getAngle().getDegrees();
        inputs.frontRightAbsoluteAngle = frontRight.getAbsoluteAngle().getRotations();
        inputs.frontRightDrivingVelocity = frontRight.getDrivingVelocity();
        inputs.frontRightTurningVelocity = frontRight.getTurningVelocity();
        inputs.frontRightDrivingAcceleration = frontRight.getDrivingAcceleration();
        inputs.frontRightTurningAcceleration = frontRight.getTurningAcceleration();
        inputs.frontRightDrivingCurrent = frontRight.getDrivingCurrent();
        inputs.frontRightTurningCurrent = frontRight.getTurningCurrent();
        inputs.frontRightDrivingVoltage = frontRight.getDrivingVoltage();
        inputs.frontRightTurningVoltage = frontRight.getTurningVoltage();
        inputs.frontRightTargetVelocity = frontRight.velocitySetpoint;
        inputs.frontRightTargetAngle = frontRight.angleSetpoint;

        // Back Left
        inputs.backLeftPosition = backLeft.getPosition();
        inputs.backLeftAngle = backLeft.getAngle().getDegrees();
        inputs.backLeftAbsoluteAngle = backLeft.getAbsoluteAngle().getRotations();
        inputs.backLeftDrivingVelocity = backLeft.getDrivingVelocity();
        inputs.backLeftTurningVelocity = backLeft.getTurningVelocity();
        inputs.backLeftDrivingAcceleration = backLeft.getDrivingAcceleration();
        inputs.backLeftDrivingAcceleration = backLeft.getTurningAcceleration();
        inputs.backLeftDrivingCurrent = backLeft.getDrivingCurrent();
        inputs.backLeftTurningCurrent = backLeft.getTurningCurrent();
        inputs.backLeftDrivingVoltage = backLeft.getDrivingVoltage();
        inputs.backLeftTurningVoltage = backLeft.getTurningVoltage();
        inputs.backLeftTargetVelocity = backLeft.velocitySetpoint;
        inputs.backLeftTargetAngle = backLeft.angleSetpoint;

        // Back Right
        inputs.backRightPosition = backRight.getPosition();
        inputs.backRightAngle = backRight.getAngle().getDegrees();
        inputs.backRightAbsoluteAngle = backRight.getAbsoluteAngle().getRotations();
        inputs.backRightDrivingVelocity = backRight.getDrivingVelocity();
        inputs.backRightTurningVelocity = backRight.getTurningVelocity();
        inputs.backRightDrivingAcceleration = backRight.getDrivingAcceleration();
        inputs.backRightTurningAcceleration = backRight.getTurningAcceleration();
        inputs.backRightDrivingCurrent = backRight.getDrivingCurrent();
        inputs.backRightTurningCurrent = backRight.getTurningCurrent();
        inputs.backRightDrivingVoltage = backRight.getDrivingVoltage();
        inputs.backRightTurningVoltage = backRight.getTurningVoltage();
        inputs.backRightTargetVelocity = backRight.velocitySetpoint;
        inputs.backRightTargetAngle = backRight.angleSetpoint;
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    @Override 
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    @Override
    public void setFrontLeftDrivingPID(double P, double I, double D) {
        frontLeft.setDrivingPID(P, I, D);
    }

    @Override
    public void setFrontLeftTurningPID(double P, double I, double D) {
        frontLeft.setTurningPID(P, I, D);
    }

    @Override
    public void setFrontRightDrivingPID(double P, double I, double D) {
        frontRight.setDrivingPID(P, I, D);
    }

    @Override
    public void setFrontRightTurningPID(double P, double I, double D) {
        frontRight.setTurningPID(P, I, D);
    }

    @Override
    public void setBackLeftDrivingPID(double P, double I, double D) {
        backLeft.setDrivingPID(P, I, D);
    }

    @Override
    public void setBackLeftTurningPID(double P, double I, double D) {
        backLeft.setTurningPID(P, I, D);
    }

    @Override
    public void setBackRightDrivingPID(double P, double I, double D) {
        backRight.setDrivingPID(P, I, D);
    }

    @Override
    public void setBackRightTurningPID(double P, double I, double D) {
        backRight.setTurningPID(P, I, D);
    }
}
