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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;


public class Swerve extends SubsystemBase {
    private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;

    // Front Left Wheel PID constants
    public double frontLeftDrivingP;
    public double frontLeftDrivingI;
    public double frontLeftDrivingD;

    public double frontLeftTurningP;
    public double frontLeftTurningI;
    public double frontLeftTurningD;

    // Front Right Wheel PID constants
    public double frontRightDrivingP;
    public double frontRightDrivingI;
    public double frontRightDrivingD;

    public double frontRightTurningP;
    public double frontRightTurningI;
    public double frontRightTurningD;

    // Back Left Wheel PID constants
    public double backLeftDrivingP;
    public double backLeftDrivingI;
    public double backLeftDrivingD;

    public double backLeftTurningP;
    public double backLeftTurningI;
    public double backLeftTurningD;

    // Back Right Wheel PID constants
    public double backRightDrivingP;
    public double backRightDrivingI;
    public double backRightDrivingD;

    public double backRightTurningP;
    public double backRightTurningI;
    public double backRightTurningD;

    private SwerveModulePosition[] modulePositions;
    private SwerveIO io;

    // Constructor
    public Swerve(SwerveIO io) {
        this.io = io;

        // Front Left Wheel PID Constants
        frontLeftDrivingP = SwerveConstants.DRIVING_PID[0][0];
        frontLeftDrivingI = SwerveConstants.DRIVING_PID[0][1];
        frontLeftDrivingD = SwerveConstants.DRIVING_PID[0][2];

        frontLeftTurningP = SwerveConstants.TURNING_PID[0][0];
        frontLeftTurningI = SwerveConstants.TURNING_PID[0][1];
        frontLeftTurningD = SwerveConstants.TURNING_PID[0][2];

        SmartDashboard.putNumber("frontLeftDrivingP", frontLeftDrivingP);
        SmartDashboard.putNumber("frontLeftDrivingI", frontLeftDrivingI);
        SmartDashboard.putNumber("frontLeftDrivingD", frontLeftDrivingD);
        SmartDashboard.putNumber("frontLeftTurningP", frontLeftTurningP);
        SmartDashboard.putNumber("frontLeftTurningI", frontLeftTurningI);
        SmartDashboard.putNumber("frontLeftTurningD", frontLeftTurningD);

        // Front Right Wheel PID Constants
        frontRightDrivingP = SwerveConstants.DRIVING_PID[1][0];
        frontRightDrivingI = SwerveConstants.DRIVING_PID[1][1];
        frontRightDrivingD = SwerveConstants.DRIVING_PID[1][2];

        frontRightTurningP = SwerveConstants.TURNING_PID[1][0];
        frontRightTurningI = SwerveConstants.TURNING_PID[1][1];
        frontRightTurningD = SwerveConstants.TURNING_PID[1][2];

        SmartDashboard.putNumber("frontRightDrivingP", frontRightDrivingP);
        SmartDashboard.putNumber("frontRightDrivingI", frontRightDrivingI);
        SmartDashboard.putNumber("frontRightDrivingD", frontRightDrivingD);
        SmartDashboard.putNumber("frontRightTurningP", frontRightTurningP);
        SmartDashboard.putNumber("frontRightTurningI", frontRightTurningI);
        SmartDashboard.putNumber("frontRightTurningD", frontRightTurningD);

        // Back Left Wheel PID Constants
        backLeftDrivingP = SwerveConstants.DRIVING_PID[2][0];
        backLeftDrivingI = SwerveConstants.DRIVING_PID[2][1];
        backLeftDrivingD = SwerveConstants.DRIVING_PID[2][2];

        backLeftTurningP = SwerveConstants.TURNING_PID[2][0];
        backLeftTurningI = SwerveConstants.TURNING_PID[2][1];
        backLeftTurningD = SwerveConstants.TURNING_PID[2][2];

        SmartDashboard.putNumber("backLeftDrivingP", backLeftDrivingP);
        SmartDashboard.putNumber("backLeftDrivingI", backLeftDrivingI);
        SmartDashboard.putNumber("backLeftDrivingD", backLeftDrivingD);
        SmartDashboard.putNumber("backLeftTurningP", backLeftTurningP);
        SmartDashboard.putNumber("backLeftTurningI", backLeftTurningI);
        SmartDashboard.putNumber("backLeftTurningD", backLeftTurningD);

        // Back Right Wheel PID Constants
        backRightDrivingP = SwerveConstants.DRIVING_PID[3][0];
        backRightDrivingI = SwerveConstants.DRIVING_PID[3][1];
        backRightDrivingD = SwerveConstants.DRIVING_PID[3][2];

        backRightTurningP = SwerveConstants.TURNING_PID[3][0];
        backRightTurningI = SwerveConstants.TURNING_PID[3][1];
        backRightTurningD = SwerveConstants.TURNING_PID[3][2];

        SmartDashboard.putNumber("backRightDrivingP", backRightDrivingP);
        SmartDashboard.putNumber("backRightDrivingI", backRightDrivingI);
        SmartDashboard.putNumber("backRightDrivingD", backRightDrivingD);
        SmartDashboard.putNumber("backRightTurningP", backRightTurningP);
        SmartDashboard.putNumber("backRightTurningI", backRightTurningI);
        SmartDashboard.putNumber("backRightTurningD", backRightTurningD);   
        
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

        // Update Front Left PID Constants for Driving
        if (frontLeftDrivingP != SmartDashboard.getNumber("frontLeftDrivingP", frontLeftDrivingP) ||
            frontLeftDrivingI != SmartDashboard.getNumber("frontLeftDrivingI", frontLeftDrivingI) ||
            frontLeftDrivingD != SmartDashboard.getNumber("frontLeftDrivingD", frontLeftDrivingD)) {
            frontLeftDrivingP = SmartDashboard.getNumber("frontLeftDrivingP", frontLeftDrivingP);
            frontLeftDrivingI = SmartDashboard.getNumber("frontLeftDrivingI", frontLeftDrivingI);
            frontLeftDrivingD = SmartDashboard.getNumber("frontLeftDrivingD", frontLeftDrivingD);
            io.setFrontLeftDrivingPID(frontLeftDrivingP, frontLeftDrivingI, frontLeftDrivingD);
        }

        // Update Front Left PID Constants for Turning
        if (frontLeftTurningP != SmartDashboard.getNumber("frontLeftTurningP", frontLeftTurningP) ||
            frontLeftTurningI != SmartDashboard.getNumber("frontLeftTurningI", frontLeftTurningI) ||
            frontLeftTurningD != SmartDashboard.getNumber("frontLeftTurningD", frontLeftTurningD)) {
            frontLeftTurningP = SmartDashboard.getNumber("frontLeftTurningP", frontLeftTurningP);
            frontLeftTurningI = SmartDashboard.getNumber("frontLeftTurningI", frontLeftTurningI);
            frontLeftTurningD = SmartDashboard.getNumber("frontLeftTurningD", frontLeftTurningD);
            io.setFrontLeftTurningPID(frontLeftTurningP, frontLeftTurningI, frontLeftTurningD);
        }

        // Update Front Right PID Constants for Driving
        if (frontRightDrivingP != SmartDashboard.getNumber("frontRightDrivingP", frontRightDrivingP) ||
            frontRightDrivingI != SmartDashboard.getNumber("frontRightDrivingI", frontRightDrivingI) ||
            frontRightDrivingD != SmartDashboard.getNumber("frontRightDrivingD", frontRightDrivingD)) {
            frontRightDrivingP = SmartDashboard.getNumber("frontRightDrivingP", frontRightDrivingP);
            frontRightDrivingI = SmartDashboard.getNumber("frontRightDrivingI", frontRightDrivingI);
            frontRightDrivingD = SmartDashboard.getNumber("frontRightDrivingD", frontRightDrivingD);
            io.setFrontRightDrivingPID(frontRightDrivingP, frontRightDrivingI, frontRightDrivingD);
        }

        // Update Front Right PID Constants for Turning
        if (frontRightTurningP != SmartDashboard.getNumber("frontRightTurningP", frontRightTurningP) ||
            frontRightTurningI != SmartDashboard.getNumber("frontRightTurningI", frontRightTurningI) ||
            frontRightTurningD != SmartDashboard.getNumber("frontRightTurningD", frontRightTurningD)) {
            frontRightTurningP = SmartDashboard.getNumber("frontRightTurningP", frontRightTurningP);
            frontRightTurningI = SmartDashboard.getNumber("frontRightTurningI", frontRightTurningI);
            frontRightTurningD = SmartDashboard.getNumber("frontRightTurningD", frontRightTurningD);
            io.setFrontRightTurningPID(frontRightTurningP, frontRightTurningI, frontRightTurningD);
        }

        // Update Back Left PID Constants for Driving
        if (backLeftDrivingP != SmartDashboard.getNumber("backLeftDrivingP", backLeftDrivingP) ||
            backLeftDrivingI != SmartDashboard.getNumber("backLeftDrivingI", backLeftDrivingI) ||
            backLeftDrivingD != SmartDashboard.getNumber("backLeftDrivingD", backLeftDrivingD)) {
            backLeftDrivingP = SmartDashboard.getNumber("backLeftDrivingP", backLeftDrivingP);
            backLeftDrivingI = SmartDashboard.getNumber("backLeftDrivingI", backLeftDrivingI);
            backLeftDrivingD = SmartDashboard.getNumber("backLeftDrivingD", backLeftDrivingD);
            io.setBackLeftDrivingPID(backLeftDrivingP, backLeftDrivingI, backLeftDrivingD);
        }

        // Update Back Left PID Constants for Turning
        if (backLeftTurningP != SmartDashboard.getNumber("backLeftTurningP", backLeftTurningP) ||
            backLeftTurningI != SmartDashboard.getNumber("backLeftTurningI", backLeftTurningI) ||
            backLeftTurningD != SmartDashboard.getNumber("backLeftTurningD", backLeftTurningD)) {
            backLeftTurningP = SmartDashboard.getNumber("backLeftTurningP", backLeftTurningP);
            backLeftTurningI = SmartDashboard.getNumber("backLeftTurningI", backLeftTurningI);
            backLeftTurningD = SmartDashboard.getNumber("backLeftTurningD", backLeftTurningD);
            io.setBackLeftTurningPID(backLeftTurningP, backLeftTurningI, backLeftTurningD);
        }

        // Update Back Right PID Constants for Driving
        if (backRightDrivingP != SmartDashboard.getNumber("backRightDrivingP", backRightDrivingP) ||
            backRightDrivingI != SmartDashboard.getNumber("backRightDrivingI", backRightDrivingI) ||
            backRightDrivingD != SmartDashboard.getNumber("backRightDrivingD", backRightDrivingD)) {
            backRightDrivingP = SmartDashboard.getNumber("backRightDrivingP", backRightDrivingP);
            backRightDrivingI = SmartDashboard.getNumber("backRightDrivingI", backRightDrivingI);
            backRightDrivingD = SmartDashboard.getNumber("backRightDrivingD", backRightDrivingD);
            io.setBackRightDrivingPID(backRightDrivingP, backRightDrivingI, backRightDrivingD);
        }

        // Update Back Right PID Constants for Turning
        if (backRightTurningP != SmartDashboard.getNumber("backRightTurningP", backRightTurningP) ||
            backRightTurningI != SmartDashboard.getNumber("backRightTurningI", backRightTurningI) ||
            backRightTurningD != SmartDashboard.getNumber("backRightTurningD", backRightTurningD)) {
            backRightTurningP = SmartDashboard.getNumber("backRightTurningP", backRightTurningP);
            backRightTurningI = SmartDashboard.getNumber("backRightTurningI", backRightTurningI);
            backRightTurningD = SmartDashboard.getNumber("backRightTurningD", backRightTurningD);
            io.setBackRightTurningPID(backRightTurningP, backRightTurningI, backRightTurningD);
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
