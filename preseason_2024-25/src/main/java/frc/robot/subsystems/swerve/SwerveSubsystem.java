package frc.robot.subsystems.swerve;

import java.io.File;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{

    private final SwerveDrive swerveDrive;


    /**
     * YAGSL initiates a swerve drive object based on the directory provided
     * @param directory Directory of swerve drive config files
     * In this case the JSON config files are found in src/main/deploy/swerve
     */
    public  SwerveSubsystem(File directory) {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // The try except is just in case YAGSL messes something up
        try{
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.SwerveConstants.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    }

    /**
     * Alternate constructor
     * 
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg){
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Constants.SwerveConstants.MAX_SPEED);
    }
    
}
