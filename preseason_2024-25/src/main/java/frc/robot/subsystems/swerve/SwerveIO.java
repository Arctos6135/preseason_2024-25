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

        public double frontLeftAbsoluteAngle;
        public double frontRightAbsoluteAngle;
        public double backLeftAbsoluteAngle;
        public double backRightAbsoluteAngle;

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

        public double frontLeftDrivingAcceleration;
        public double frontRightDrivingAcceleration;
        public double backLeftDrivingAcceleration;
        public double backRightDrivingAcceleration;

        public double frontLeftTurningAcceleration;
        public double frontRightTurningAcceleration;
        public double backLeftTurningAcceleration;
        public double backRightTurningAcceleration;

        public double frontLeftDrivingCurrent;
        public double frontRightDrivingCurrent;
        public double backLeftDrivingCurrent;
        public double backRightDrivingCurrent;

        public double frontLeftTurningCurrent;
        public double frontRightTurningCurrent;
        public double backLeftTurningCurrent;
        public double backRightTurningCurrent;

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
        return new Rotation2d(90);
    }

    public void resetGyro(){}

    // Placeholder which gets overidden.
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[3];
    }

    public void resetAbsoluteEncoders() {}

    public void setFrontLeftDrivingPID(double P, double I, double D) {}

    public void setFrontLeftTurningPID(double P, double I, double D) {}

    public void setFrontRightDrivingPID(double P, double I, double D) {}

    public void setFrontRightTurningPID(double P, double I, double D) {}

    public void setBackLeftDrivingPID(double P, double I, double D) {}

    public void setBackLeftTurningPID(double P, double I, double D) {}

    public void setBackRightDrivingPID(double P, double I, double D) {}

    public void setBackRightTurningPID(double P, double I, double D) {}

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{};
    }

    public void setDrivingVoltages(double voltage) {}

    public void update() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    };
}
