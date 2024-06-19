package frc.robot.subsystems.drivetrain;

// Imports the advantage scope autologger.
import org.littletonrobotics.junction.AutoLog;

/**
 * This class represents anything that behaves like a drivetrain. By channeling all our interactions with the drivetrain hardware through this class, 
 * we can have both the real robot and the simulated robot be "something that behaves like a drivetrain", allowing us to easily swap between them.
 */
public class DrivetrainIO {
    
    @Autolog
    public static class DrivingInputs {
        // Position in meters.
        public double frontLeftPosition;
        public double frontRightPosition;
        public double backLeftPosition;
        public double backRightPosition;

        // Velocity in meters per second.
        public double frontLeftVelocity;
        public double frontRightVelocity;
        public double backLeftPosition;
        public double backRightPosition;

        // Yaw in radians.
        public double yaw;
        public double yawRate;

        // Current in amperes.
        public double frontLeftCurrent;
        public double frontRightCurrent;
        public double backLeftPosition;
        public double backRightPosition;

        // Temperature in celcius.
        public double frontLeftTemperature;
        public double frontRightTemperature;
        public double backLeftTemperature;
        public double backRightTemperature;

        // Max voltage (percentage).
        public double frontLeftVoltage;
        public double frontRightVoltage;
        public double backLeftVoltage;
        public double backRightVoltage;
    }

    /**
     * Update the sensor data.
     * @param inputs the sensor data
     */
     public void updateInputs(DrivetrainInputs inputs) {}

     /**
      * Set the voltages of the drive motors from [-12, 12]
      * @param frontLeft front left motor voltage
      * @param frontRight front right motor voltage
      * @param backLeft back left motor voltage
      * @param backRight back right motor voltage
      */
      public void setVoltages(double frontLeft, double frontRight, double backLeft, double backRight) {}

      /**
       * Set the speeds of the drive motors.
       * @param frontLeft front left motor target speed
       * @param frontRight front right motor target speed
       * @param backLeft back left motor target speed
       * @param backRight back right motor target speed
       */
      public void setSpeeds(frontLeft, frontRight, backLeft, backRight) {}
}
