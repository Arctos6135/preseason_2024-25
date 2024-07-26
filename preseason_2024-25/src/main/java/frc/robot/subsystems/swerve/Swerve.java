package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;


public class Swerve extends SubsystemBase {
    private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;

    public double[] frontLeftDrivingPID = SwerveConstants.FRONT_LEFT_DRIVING_PID;
    public double[] frontRightDrivingPID = SwerveConstants.FRONT_RIGHT_DRIVING_PID;
    public double[] backLeftDrivingPID = SwerveConstants.BACK_LEFT_DRIVING_PID;
    public double[] backRightDrivingPID = SwerveConstants.BACK_RIGHT_DRIVING_PID;

    public double[] frontLeftTurningPID = SwerveConstants.FRONT_LEFT_TURNING_PID;
    public double[] frontRightTurningPID = SwerveConstants.FRONT_RIGHT_TURNING_PID;
    public double[] backLeftTurningPID = SwerveConstants.BACK_LEFT_TURNING_PID;
    public double[] backRightTurningPID = SwerveConstants.BACK_RIGHT_TURNING_PID;

    private SwerveModulePosition[] modulePositions;
    private SwerveIO io;

    // Constructor
    public Swerve(SwerveIO io) {
        this.io = io;

        SmartDashboard.putNumberArray("frontLeftDrivingPID", frontLeftDrivingPID);
        SmartDashboard.putNumberArray("frontRightDrivingPID", frontRightDrivingPID);
        SmartDashboard.putNumberArray("backLeftDrivingPID", backLeftDrivingPID);
        SmartDashboard.putNumberArray("backRightDrivingPID", backRightDrivingPID);

        SmartDashboard.putNumberArray("frontLeftTurningPID", frontLeftTurningPID);
        SmartDashboard.putNumberArray("frontRightTurningPID", frontRightTurningPID);
        SmartDashboard.putNumberArray("backLeftTurningPID", backLeftTurningPID);
        SmartDashboard.putNumberArray("backRightTurningPID", backRightTurningPID);
        
        // Create a swerve module positions object.
        modulePositions = io.getModulePositions();
        // Create SwerveDriveKinematics object
        // Given units are the x and y distances of the wheel to the center of robot.
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.DISTANCE_TO_CENTER, SwerveConstants.DISTANCE_TO_CENTER), // Front Left
            new Translation2d(SwerveConstants.DISTANCE_TO_CENTER, -SwerveConstants.DISTANCE_TO_CENTER), // Front Right
            new Translation2d(-SwerveConstants.DISTANCE_TO_CENTER, SwerveConstants.DISTANCE_TO_CENTER), // Back Left
            new Translation2d(-SwerveConstants.DISTANCE_TO_CENTER, -SwerveConstants.DISTANCE_TO_CENTER) // Back Right
        );

        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0), modulePositions);

        io.setStates(new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        }
        );
    }

    public void drive(double x, double y, double rotation){
        // Converts entered values (-1 to 1) into the units used by drivetrain

        double xSpeed = x * SwerveConstants.MAX_SPEED;
        double ySpeed = y * SwerveConstants.MAX_SPEED;
        double rSpeed = rotation * SwerveConstants.MAX_ANGULAR_VELOCITY;

        Rotation2d gyroRotation = io.getAngle();

        var swerveModuleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rSpeed,
                gyroRotation
            )
        );

        io.setStates(swerveModuleStates);
        }

    @Override
    public void periodic() {
        SmartDashboard.updateValues();

        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);

        if (frontLeftDrivingPID != SmartDashboard.getNumberArray("frontLeftDrivingPID", frontLeftDrivingPID)) {
            frontLeftDrivingPID = SmartDashboard.getNumberArray("frontLeftDrivingPID", frontLeftDrivingPID);
            io.setFrontLeftDrivingPID(frontLeftDrivingPID[0], frontLeftDrivingPID[1], frontLeftDrivingPID[2]);
        }
        if (frontRightDrivingPID != SmartDashboard.getNumberArray("frontRightDrivingPID", frontRightDrivingPID)) {
            frontRightDrivingPID = SmartDashboard.getNumberArray("frontRightDrivingPID", frontRightDrivingPID);
            io.setFrontRightDrivingPID(frontRightDrivingPID[0], frontRightDrivingPID[1], frontRightDrivingPID[2]);
        }
        if (backLeftDrivingPID != SmartDashboard.getNumberArray("backLeftDrivingPID", backLeftDrivingPID)) {
            backLeftDrivingPID = SmartDashboard.getNumberArray("backLeftDrivingPID", backLeftDrivingPID);
            io.setBackLeftDrivingPID(backLeftDrivingPID[0], backLeftDrivingPID[1], backLeftDrivingPID[2]);
        }
        if (backRightDrivingPID != SmartDashboard.getNumberArray("backRightDrivingPID", backRightDrivingPID)) {
            backRightDrivingPID = SmartDashboard.getNumberArray("backRightDrivingPID", backRightDrivingPID);
            io.setBackRightDrivingPID(backRightDrivingPID[0], backRightDrivingPID[1], backRightDrivingPID[2]);
        }

        if (frontLeftTurningPID != SmartDashboard.getNumberArray("frontLeftTurningPID", frontLeftTurningPID)) {
            frontLeftTurningPID = SmartDashboard.getNumberArray("frontLeftTurningPID", frontLeftTurningPID);
            io.setFrontLeftTurningPID(frontLeftTurningPID[0], frontLeftTurningPID[1], frontLeftTurningPID[2]);
        }
        if (frontRightTurningPID != SmartDashboard.getNumberArray("frontRightTurningPID", frontRightTurningPID)) {
            frontRightTurningPID = SmartDashboard.getNumberArray("frontRightTurningPID", frontRightTurningPID);
            io.setFrontRightTurningPID(frontRightTurningPID[0], frontRightTurningPID[1], frontRightTurningPID[2]);
        }
        if (backLeftTurningPID != SmartDashboard.getNumberArray("backLeftTurningPID", backLeftTurningPID)) {
            backLeftTurningPID = SmartDashboard.getNumberArray("backLeftTurningPID", backLeftTurningPID);
            io.setBackLeftTurningPID(backLeftTurningPID[0], backLeftTurningPID[1], backLeftTurningPID[2]);
        }
        if (backRightTurningPID != SmartDashboard.getNumberArray("backRightTurningPID", backRightTurningPID)) {
            backRightTurningPID = SmartDashboard.getNumberArray("backRightTurningPID", backRightTurningPID);
            io.setBackRightTurningPID(backRightTurningPID[0], backRightTurningPID[1], backRightTurningPID[2]);
        }

        // Updates the odometry.
        odometry.update(
            io.getAngle(),
            io.getModulePositions()
        );

        Logger.recordOutput("pose", odometry.getPoseMeters());
    }

    /**
     * Returns the current pose of the robot.
     * 
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetStates() {
        io.updateInputs(inputs);

        io.setStates(new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        });
    }
}
