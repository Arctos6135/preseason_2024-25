package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    
    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_ANGULAR_OFFSET = Math.PI;
    public static final double BACK_RIGHT_ANGULAR_OFFSET = Math.PI / 2;

    public static final double DRIVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0);

    public static final int TURNING_CURRENT_LIMIT = 40;

    // In radians / radians per second.
    public static final double TURNING_ENCODER_POSITION_FACTOR = 2 * Math.PI;
    public static final double TURNING_VELOCITY_CONVERSION = 0;

    public static final double DRIVING_CURRENT_LIMIT = 0;

    // In meters.
    public static final double DRIVING_ENCODER_POSITION_FACTOR = DRIVE_WHEEL_CIRCUMFERENCE * 2 * Math.PI;

    // This won't cause any issues
    public static final double MAX_SPEED = 100000;
}
