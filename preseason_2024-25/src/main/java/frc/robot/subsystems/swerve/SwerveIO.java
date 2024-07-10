package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveIO {
    @AutoLog
    public static class SwerveInputs {
        public Pose2d pose;

        public double frontLeftAngle;
        public double frontRightAngle;
        public double backLeftAngle;
        public double backRightAngle;

        public double frontLeftPosition;
        public double frontRightPosition;
        public double backLeftPosition;
        public double backRightPosition;

        public double frontLeftDrivingVelocity;
        public double frontRightDrivingVelocity;
        public double backLeftDrivingVelocity;
        public double backRightDrivingVelocity;

        public double frontLeftTurningVelocity;
        public double frontRightTurningVelocity;
        public double backLeftTurningVelocity;
        public double backRightTurningVelocity;

        public double frontLeftDrivingVoltage;
        public double frontRightDrivingVoltage;
        public double backLeftDrivingVoltage;
        public double backRightDrivingVoltage;

        public double frontLeftTurningVoltage;
        public double frontRightTurningVoltage;
        public double backLeftTurningVoltage;
        public double backRightTurningVoltage;

        public double frontLeftTargetAngle;
        public double frontRightTargetAngle;
        public double backLeftTargetAngle;
        public double backRightTargetAngle;

        public double frontLeftTargetVelocity;
        public double frontRightTargetVelocity;
        public double backLeftTargetVelocity;
        public double backRightTargetVelocity;
    }

    public void setState(SwerveModuleState desiredState) {}

    public void updateInputs(SwerveInputs inputs) {}

    public void setStates(SwerveModuleState[] moduleStates) {}

    public Rotation2d getAngle() {
        return new Rotation2d(0);
    }

    // Placeholder which gets ovveridden.
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[3];
    }
}
