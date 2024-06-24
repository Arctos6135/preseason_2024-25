package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;


public class Swerve extends SubsystemBase {
    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    AHRS gyro;

    private final SwerveModule frontLeftModule = new SwerveModule(
        SwerveConstants.FRONT_LEFT_DRIVE,
        SwerveConstants.FRONT_LEFT_TURN,
        SwerveConstants.FRONT_LEFT_ENCODER
    );

    // Constructor
    public Swerve() {

        // Create SwerveDriveKinematics object
        // Given units are the x and y distances of the wheel to the center of robot.
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.DISTANCE_TO_CENTER, SwerveConstants.DISTANCE_TO_CENTER), // Front Left
            new Translation2d(SwerveConstants.DISTANCE_TO_CENTER, -SwerveConstants.DISTANCE_TO_CENTER), // Front Right
            new Translation2d(-SwerveConstants.DISTANCE_TO_CENTER, SwerveConstants.DISTANCE_TO_CENTER), // Back Left
            new Translation2d(-SwerveConstants.DISTANCE_TO_CENTER, -SwerveConstants.DISTANCE_TO_CENTER) // Back Right
        );
    }
}
