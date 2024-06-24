package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.reduxrobotics.canand.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANBusConstants;
import frc.robot.constants.SwerveConstants;


public class Swerve extends SubsystemBase {
    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    AHRS gyro;
    // Constructor
    public Swerve() {

        private final SwerveModule frontLeftModule = new SwerveModule(CANBusConstants.FRONT_LEFT_DRIVE, CANBusConstants.FRONT_LEFT_TURN, SwerveConstants.FRONT_LEFT_ANGULAR_OFFSET);
        private final SwerveModule frontRightModule = new SwerveModule(CANBusConstants.FRONT_RIGHT_DRIVE, CANBusConstants.FRONT_RIGHT_TURN, SwerveConstants.FRONT_RIGHT_ANGULAR_OFFSET);
        private final SwerveModule backLeftModule = new SwerveModule(CANBusConstants.BACK_LEFT_DRIVE, CANBusConstants.BACK_LEFT_TURN, SwerveConstants.BACK_LEFT_ANGULAR_OFFSET);
        private final SwerveModule backRightModule = new SwerveModule(CANBusConstants.BACK_RIGHT_DRIVE, CANBusConstants.BACK_RIGHT_TURN, SwerveConstants.BACK_RIGHT_ANGULAR_OFFSET);

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
