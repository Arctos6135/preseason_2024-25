package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANBusConstants;
import frc.robot.constants.SwerveConstants;


public class Swerve extends SubsystemBase {
    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    private final AHRS gyro;

    SwerveModulePosition[] modulePositions;

    private final SwerveModule frontLeftModule;

    private final SwerveModule frontRightModule;

    private final SwerveModule backLeftModule;

    private final SwerveModule backRightModule;

    // Constructor
    public Swerve() {
        gyro = new AHRS();

        frontLeftModule  = new SwerveModule(
            CANBusConstants.FRONT_LEFT_DRIVE,
            CANBusConstants.FRONT_LEFT_TURN,
            SwerveConstants.FRONT_LEFT_ENCODER_PORT,
            SwerveConstants.FRONT_LEFT_ANGULAR_OFFSET
        );

        frontRightModule  = new SwerveModule(
            CANBusConstants.FRONT_RIGHT_DRIVE, 
            CANBusConstants.FRONT_RIGHT_TURN,
            SwerveConstants.FRONT_RIGHT_ENCODER_PORT,
            SwerveConstants.FRONT_RIGHT_ANGULAR_OFFSET
        );

        backLeftModule  = new SwerveModule(
            CANBusConstants.BACK_LEFT_DRIVE, 
            CANBusConstants.BACK_LEFT_TURN, 
            SwerveConstants.BACK_LEFT_ENCODER_PORT,
            SwerveConstants.BACK_LEFT_ANGULAR_OFFSET
        );

        backRightModule  = new SwerveModule(
            CANBusConstants.BACK_RIGHT_DRIVE, 
            CANBusConstants.BACK_RIGHT_TURN,
            SwerveConstants.BACK_RIGHT_ENCODER_PORT,
            SwerveConstants.BACK_RIGHT_ANGULAR_OFFSET
        );
        
        // Create a swerve module positions object.
        modulePositions = new SwerveModulePosition[] {
            frontLeftModule.getModulePosition(),
            frontRightModule.getModulePosition(),
            backLeftModule.getModulePosition(),
            backRightModule.getModulePosition()
        };

        // Create SwerveDriveKinematics object
        // Given units are the x and y distances of the wheel to the center of robot.
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.DISTANCE_TO_CENTER, SwerveConstants.DISTANCE_TO_CENTER), // Front Left
            new Translation2d(SwerveConstants.DISTANCE_TO_CENTER, -SwerveConstants.DISTANCE_TO_CENTER), // Front Right
            new Translation2d(-SwerveConstants.DISTANCE_TO_CENTER, SwerveConstants.DISTANCE_TO_CENTER), // Back Left
            new Translation2d(-SwerveConstants.DISTANCE_TO_CENTER, -SwerveConstants.DISTANCE_TO_CENTER) // Back Right
        );

        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getAngle()), modulePositions);
    }

    public void drive(double x, double y, double rotation){
        // Converts entered values (-1 to 1) into the units used by drivetrain

        double xSpeed = x * SwerveConstants.MAX_SPEED;
        double ySpeed = y * SwerveConstants.MAX_SPEED;
        double rSpeed = rotation * SwerveConstants.MAX_ANGULAR_VELOCITY;

        Rotation2d gyroRotation = gyro.getRotation2d();

        var swerveModuleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rSpeed,
                gyroRotation
            )
        );

        frontLeftModule.setState(swerveModuleStates[0]);
        frontRightModule.setState(swerveModuleStates[1]);
        backLeftModule.setState(swerveModuleStates[2]);
        backRightModule.setState(swerveModuleStates[3]);
    }

    @Override
    public void periodic() {
        // Updates the odometry.
        odometry.update(
            Rotation2d.fromDegrees(gyro.getAngle()),
            new SwerveModulePosition[] {
                frontLeftModule.getModulePosition(),
                frontRightModule.getModulePosition(),
                backLeftModule.getModulePosition(),
                backRightModule.getModulePosition()
            }
        );

        Logger.recordOutput("pose", getPose());
    }

    /**
     * Returns the current pose of the robot.
     * 
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
}
