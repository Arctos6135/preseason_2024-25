package frc.robot.subsystems.swerve;

import java.sql.Driver;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.SwerveConstants;
import frc.robot.util.TunableDouble;
import swervelib.SwerveDrive;


public class Swerve extends SubsystemBase {
    private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    public Rotation2d gyroRotation = Rotation2d.fromDegrees(0);

    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;

    private SwerveModulePosition[] modulePositions;
    private SwerveDriveWheelPositions intialModulePositions;

    private Field2d field;

    private SwerveIO io;

    // Constructor
    public Swerve(SwerveIO io) {
        this.io = io;

        // Create a swerve module positions object.
        modulePositions = io.getModulePositions();
        intialModulePositions = new SwerveDriveWheelPositions(modulePositions);

        field = new Field2d();

        SmartDashboard.putData("field", field);

        // Create SwerveDriveKinematics object
        // Given units are the x and y distances of the wheel to the center of robot.
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.DISTANCE_TO_CENTER, SwerveConstants.DISTANCE_TO_CENTER), // Front Left
            new Translation2d(SwerveConstants.DISTANCE_TO_CENTER, -SwerveConstants.DISTANCE_TO_CENTER), // Front Right
            new Translation2d(-SwerveConstants.DISTANCE_TO_CENTER, SwerveConstants.DISTANCE_TO_CENTER), // Back Left
            new Translation2d(-SwerveConstants.DISTANCE_TO_CENTER, -SwerveConstants.DISTANCE_TO_CENTER) // Back Right
        );

        kinematics.resetHeadings(io.getModulePositions()[0].angle, 
        io.getModulePositions()[1].angle,
        io.getModulePositions()[2].angle,
        io.getModulePositions()[3].angle);

        odometry = new SwerveDriveOdometry(kinematics, getAngle(), modulePositions);

        io.setStates(
            io.getModuleStates()
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

    public void setDrivingVoltage(double voltage) {
        io.setDrivingVoltages(voltage);
    }

    public void robotRelativeDrive(ChassisSpeeds chassisSpeeds) {

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            chassisSpeeds.omegaRadiansPerSecond *= -1;
        }

        var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, SwerveConstants.MAX_SPEED);

        io.setStates(swerveModuleStates);

        Logger.recordOutput("target states", swerveModuleStates);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(io.getModuleStates()), io.getAngle());
    }

    public Rotation2d getAngle() {
        return io.getAngle();
    }

    public void drive(double x, double y, double rotation){
        // Converts entered values (-1 to 1) into the units used by drivetrain

        double xSpeed = x * SwerveConstants.MAX_SPEED;
        double ySpeed = y * SwerveConstants.MAX_SPEED;

        double rSpeed = rotation * SwerveConstants.MAX_ANGULAR_VELOCITY;

        if (DriverStation.getRawAllianceStation() == AllianceStationID.Blue1 || DriverStation.getRawAllianceStation() == AllianceStationID.Blue2 || DriverStation.getRawAllianceStation() == AllianceStationID.Blue3) {
            xSpeed *= 1;
            ySpeed *= 1;
        }
        else {
            xSpeed *= -1;
            ySpeed *= -1;
        }

        var swerveModuleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rSpeed,
                getAngle()
            )
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);

        io.setStates(swerveModuleStates);

        Logger.recordOutput("target states", swerveModuleStates);
        }

    @Override
    public void periodic() {
        SmartDashboard.updateValues();
        io.updateInputs(inputs);
        // io.update();

        modulePositions = io.getModulePositions();

        gyroRotation = Rotation2d.fromRadians(kinematics.toTwist2d(intialModulePositions, new SwerveDriveWheelPositions(modulePositions)).dtheta);

        // Updates the odometry.
        odometry.update(getAngle(), modulePositions);

        field.setRobotPose(getPose());

        Logger.recordOutput("pose", odometry.getPoseMeters());
        Logger.recordOutput("states", io.getModuleStates());

        Logger.processInputs(getName(), inputs);
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
        

        odometry.resetPosition(getAngle(), io.getModulePositions(), pose);
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
