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

    public double dashboardDrivingP;
    public double dashboardDrivingI;
    public double dashboardDrivingD;

    public double dashboardTurningP;
    public double dashboardTurningI;
    public double dashboardTurningD;

    private SwerveModulePosition[] modulePositions;
    private SwerveIO io;

    // Constructor
    public Swerve(SwerveIO io) {
        this.io = io;

        dashboardDrivingP = SwerveConstants.DRIVING_PID[0][0];
        dashboardDrivingI = SwerveConstants.DRIVING_PID[0][1];
        dashboardDrivingD = SwerveConstants.DRIVING_PID[0][2];

        SmartDashboard.putNumber("dashboardDrivingP", dashboardDrivingP);
        SmartDashboard.putNumber("dashboardDrivingI", dashboardDrivingI);
        SmartDashboard.putNumber("dashboardDrivingD", dashboardDrivingD);

        dashboardTurningP = SwerveConstants.TURNING_PID[0][0];
        dashboardTurningI = SwerveConstants.TURNING_PID[0][1];
        dashboardTurningD = SwerveConstants.TURNING_PID[0][2];

        SmartDashboard.putNumber("dashboardTurningP", dashboardTurningP);
        SmartDashboard.putNumber("dashboardTurningI", dashboardTurningI);
        SmartDashboard.putNumber("dashboardTurningD", dashboardDrivingD);
        
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

        // Updates the odometry.
        odometry.update(
            io.getAngle(),
            io.getModulePositions()
        );

        Logger.recordOutput("pose", odometry.getPoseMeters());

        if (dashboardDrivingP != SmartDashboard.getNumber("dashboardDrivingP", dashboardDrivingP) || dashboardDrivingI != SmartDashboard.getNumber("dashboardDrivingI", dashboardDrivingI) || dashboardDrivingD != SmartDashboard.getNumber("dashboardDrivingD", dashboardDrivingD)) {
            dashboardDrivingP = SmartDashboard.getNumber("dashboardDrivingP", dashboardDrivingP);
            dashboardDrivingI = SmartDashboard.getNumber("dashboardDrivingI", dashboardDrivingI);
            dashboardDrivingD = SmartDashboard.getNumber("dashboardDrivingD", dashboardDrivingD);
            io.setDrivingPID(dashboardDrivingP, dashboardDrivingI, dashboardDrivingD);
            resetStates();
        }

        if (dashboardTurningP != SmartDashboard.getNumber("dashboardTurningP", dashboardTurningP) || dashboardTurningI != SmartDashboard.getNumber("dashboardTurningI", dashboardTurningI) || dashboardTurningD != SmartDashboard.getNumber("dashboardTurningD", dashboardTurningD)) {
            dashboardTurningP = SmartDashboard.getNumber("dashboardTurningP", dashboardTurningP);
            dashboardTurningI = SmartDashboard.getNumber("dashboardTurningI", dashboardTurningI);
            dashboardTurningD = SmartDashboard.getNumber("dashboardTurningD", dashboardTurningD);
            io.setTurningPID(dashboardTurningP, dashboardTurningI, dashboardTurningD);
            resetStates();
        }
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
