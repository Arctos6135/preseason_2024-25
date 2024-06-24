package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Swerve extends SubsystemBase {
    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    AHRS gyro;
    // Constructor
    public Swerve() {

        // Create SwerveDriveKinematics object
        // Given units are the x and y distances of the wheel to the center of robot.
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(15.0), Units.inchesToMeters(15.0)), // Front Left
            new Translation2d(Units.inchesToMeters(15.0), Units.inchesToMeters(-15.0)), // Front Right
            new Translation2d(Units.inchesToMeters(-15.0), Units.inchesToMeters(15.0)), // Back Left
            new Translation2d(Units.inchesToMeters(-15.0), Units.inchesToMeters(-15.0)) // Back Right
        );
    }
}
