package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.util.TunableDouble;


public class Swerve extends SubsystemBase {
    private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;

    // Front Left Wheel PID constants
    public TunableDouble frontLeftDrivingP;
    public TunableDouble frontLeftDrivingI;
    public TunableDouble frontLeftDrivingD;

    public TunableDouble frontLeftTurningP;
    public TunableDouble frontLeftTurningI;
    public TunableDouble frontLeftTurningD;

    // Front Right Wheel PID constants
    public TunableDouble frontRightDrivingP;
    public TunableDouble frontRightDrivingI;
    public TunableDouble frontRightDrivingD;

    public TunableDouble frontRightTurningP;
    public TunableDouble frontRightTurningI;
    public TunableDouble frontRightTurningD;

    // Back Left Wheel PID constants
    public TunableDouble backLeftDrivingP;
    public TunableDouble backLeftDrivingI;
    public TunableDouble backLeftDrivingD;

    public TunableDouble backLeftTurningP;
    public TunableDouble backLeftTurningI;
    public TunableDouble backLeftTurningD;

    // Back Right Wheel PID constants
    public TunableDouble backRightDrivingP;
    public TunableDouble backRightDrivingI;
    public TunableDouble backRightDrivingD;

    public TunableDouble backRightTurningP;
    public TunableDouble backRightTurningI;
    public TunableDouble backRightTurningD;

    private SwerveModulePosition[] modulePositions;
    private SwerveIO io;

    // Constructor
    public Swerve(SwerveIO io) {
        this.io = io;

        // Front Left Wheel PID Constants
        frontLeftDrivingP = new TunableDouble("frontLeftDrivingP", SwerveConstants.DRIVING_PID[0][0]);
        frontLeftDrivingI = new TunableDouble("frontLeftDrivingI", SwerveConstants.DRIVING_PID[0][1]);
        frontLeftDrivingD = new TunableDouble("frontLeftDrivingD", SwerveConstants.DRIVING_PID[0][2]);

        // Front Right Wheel PID Constants
        frontRightDrivingP = new TunableDouble("frontRightDrivingP", SwerveConstants.DRIVING_PID[1][0]);
        frontRightDrivingI = new TunableDouble("frontRightDrivingI", SwerveConstants.DRIVING_PID[1][1]);
        frontRightDrivingD = new TunableDouble("frontRightDrivingD", SwerveConstants.DRIVING_PID[1][2]);   
        
        // Back Left Wheel PID Constants
        backLeftDrivingP = new TunableDouble("backLeftDrivingP", SwerveConstants.DRIVING_PID[2][0]);
        backLeftDrivingI = new TunableDouble("backLeftDrivingI", SwerveConstants.DRIVING_PID[2][1]);
        backLeftDrivingD = new TunableDouble("backLeftDrivingD", SwerveConstants.DRIVING_PID[2][2]);

        // Back Right Wheel PID Constants
        backRightDrivingP = new TunableDouble("backRightDrivingP", SwerveConstants.DRIVING_PID[3][0]);
        backRightDrivingI = new TunableDouble("backRightDrivingI", SwerveConstants.DRIVING_PID[3][1]);
        backRightDrivingD = new TunableDouble("backRightDrivingD", SwerveConstants.DRIVING_PID[3][2]);

        // Front Left Wheel PID Constants
        frontLeftTurningP = new TunableDouble("frontLeftTurningP", SwerveConstants.TURNING_PID[0][0]);
        frontLeftTurningI = new TunableDouble("frontLeftTurningI", SwerveConstants.TURNING_PID[0][1]);
        frontLeftTurningD = new TunableDouble("frontLeftTurningD", SwerveConstants.TURNING_PID[0][2]);

        // Front Right Wheel PID Constants
        frontRightTurningP = new TunableDouble("frontRightTurningP", SwerveConstants.TURNING_PID[1][0]);
        frontRightTurningI = new TunableDouble("frontRightTurningI", SwerveConstants.TURNING_PID[1][1]);
        frontRightTurningD = new TunableDouble("frontRightTurningD", SwerveConstants.TURNING_PID[1][2]);   
        
        // Back Left Wheel PID Constants
        backLeftTurningP = new TunableDouble("backLeftTurningP", SwerveConstants.TURNING_PID[2][0]);
        backLeftTurningI = new TunableDouble("backLeftTurningI", SwerveConstants.TURNING_PID[2][1]);
        backLeftTurningD = new TunableDouble("backLeftTurningD", SwerveConstants.TURNING_PID[2][2]);

        // Back Right Wheel PID Constants
        backRightTurningP = new TunableDouble("backRightTurningP", SwerveConstants.TURNING_PID[3][0]);
        backRightTurningI = new TunableDouble("backRightTurningI", SwerveConstants.TURNING_PID[3][1]);
        backRightTurningD = new TunableDouble("backRightTurningD", SwerveConstants.TURNING_PID[3][2]);


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

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetPose, 
            this::getChassisSpeeds, 
            this::robotRelativeDrive, 
            SwerveConstants.autoConfig, 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);
    }

    /**
     * Reset the values of the absolute encoders to be zero.
     */
    public void resetAbsoluteEncoders() {
        io.resetAbsoluteEncoders();
    }

    public void resetGyro(){
        io.resetGyro();
    }

    public void setDrivingVoltage(double voltage) {
        io.setDrivingVoltages(voltage);
    }

    public void robotRelativeDrive(ChassisSpeeds chassisSpeeds) {
        var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, SwerveConstants.MAX_SPEED);

        io.setStates(swerveModuleStates);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(io.getModuleStates());
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
        if (frontLeftDrivingP.update() || frontLeftDrivingI.update() || frontLeftDrivingD.update()) {
            io.setFrontLeftDrivingPID(frontLeftDrivingP.get(), frontLeftDrivingI.get(), frontLeftDrivingD.get());
        }

        // Update Front Left PID Constants for Turning
        if (frontLeftTurningP.update() || frontLeftTurningI.update() || frontLeftTurningD.update()) {
            io.setFrontLeftTurningPID(frontLeftTurningP.get(), frontLeftTurningI.get(), frontLeftTurningD.get());
        }

        // Update Front Right PID Constants for Driving
        if (frontRightDrivingP.update() || frontRightDrivingI.update() || frontRightDrivingD.update()) {
            io.setFrontRightDrivingPID(frontRightDrivingP.get(), frontRightDrivingI.get(), frontRightDrivingD.get());
        }

        // Update Front Right PID Constants for Turning
        if (frontRightTurningP.update() || frontRightTurningI.update() || frontRightTurningD.update()) {
            io.setFrontRightTurningPID(frontRightTurningP.get(), frontRightTurningI.get(), frontRightTurningD.get());
        }

        // Update Back Left PID Constants for Driving
        if (backLeftDrivingP.update() || backLeftDrivingI.update() || backLeftDrivingD.update()) {
            io.setBackLeftDrivingPID(backLeftDrivingP.get(), backLeftDrivingI.get(), backLeftDrivingD.get());
        }

        // Update Back Left PID Constants for Turning
        if (backLeftTurningP.update() || backLeftTurningI.update() || backLeftTurningD.update()) {
            io.setBackLeftTurningPID(backLeftTurningP.get(), backLeftTurningI.get(), backLeftTurningD.get());
        }

        // Update Back Right PID Constants for Driving
        if (backRightDrivingP.update() || backRightDrivingI.update() || backRightDrivingD.update()) {
            io.setBackRightDrivingPID(backRightDrivingP.get(), backRightDrivingI.get(), backRightDrivingD.get());
        }

        // Update Back Right PID Constants for Turning
        if (backRightTurningP.update() || backRightTurningI.update() || backRightTurningD.update()) {
            io.setBackRightTurningPID(backRightTurningP.get(), backRightTurningI.get(), backRightTurningD.get());
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

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(io.getAngle(), io.getModulePositions(), pose);
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
