package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    /* Assume units are in
     * - Meters
     * - Seconds
     * - Radians
     * Unless otherwise specified
     */

    public static final int FRONT_LEFT_ENCODER_PORT = 0;
    public static final int FRONT_RIGHT_ENCODER_PORT = 1;
    public static final int BACK_LEFT_ENCODER_PORT = 2;
    public static final int BACK_RIGHT_ENCODER_PORT = 3;
    public static final List<Integer> ENCODER_PORTS = new ArrayList<>() {{
        add(FRONT_LEFT_ENCODER_PORT);
        add(FRONT_RIGHT_ENCODER_PORT);
        add(BACK_LEFT_ENCODER_PORT);
        add(BACK_RIGHT_ENCODER_PORT);
    }};
    
    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_ANGULAR_OFFSET = 0.0;
    public static final double FRONT_RIGHT_ANGULAR_OFFSET = 0.0;
    public static final double BACK_LEFT_ANGULAR_OFFSET = 0.0;
    public static final double BACK_RIGHT_ANGULAR_OFFSET = 0.0;
    public static final List<Double> ANGULAR_OFFSETS = new ArrayList<>() {{
        add(FRONT_LEFT_ANGULAR_OFFSET);
        add(FRONT_RIGHT_ANGULAR_OFFSET);
        add(BACK_LEFT_ANGULAR_OFFSET);
        add(BACK_RIGHT_ANGULAR_OFFSET);
    }};

    // Position offsets (rotations).
    public static final double FRONT_LEFT_POSITION_OFFSET = 0.935;
    public static final double FRONT_RIGHT_POSITION_OFFSET = 0.570;
    public static final double BACK_LEFT_POSITION_OFFSET = 0.510;
    public static final double BACK_RIGHT_POSITION_OFFSET = 0.186;
    public static double[] POSITION_OFFSETS = {
        FRONT_LEFT_POSITION_OFFSET,
        FRONT_RIGHT_POSITION_OFFSET,
        BACK_LEFT_POSITION_OFFSET,
        BACK_RIGHT_POSITION_OFFSET
    };

    // This of course assumes we manage to actually get the COG in the middle
    public static final double DISTANCE_TO_CENTER = Units.inchesToMeters(12.5);

    public static final double DRIVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0 * Math.PI);

    public static final int TURNING_CURRENT_LIMIT = 40;
    public static final int DRIVING_CURRENT_LIMIT = 60;

    public static final double DRIVING_GEARING_RATIO = 6.12;
    public static final double TURNING_GEARING_RATIO = 150.0 / 7.0;

    // In radians / radians per second.
    public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI) / TURNING_GEARING_RATIO;
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = TURNING_ENCODER_POSITION_FACTOR / 60.0;

    // In meters.
    public static final double DRIVING_ENCODER_POSITION_FACTOR = DRIVE_WHEEL_CIRCUMFERENCE / DRIVING_GEARING_RATIO;
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = (DRIVE_WHEEL_CIRCUMFERENCE / DRIVING_GEARING_RATIO) / 60.0;

    public static final double MAX_SPEED = 4;
    public static final double MAX_ANGULAR_VELOCITY = 4;

    // Gains for feedforward and simulation purposes.
    public static final double[] FRONT_LEFT_TURNING_GAINS = {0.135, 0.002209322474, 0.0008064027032};
    public static final double[] FRONT_RIGHT_TURNING_GAINS = {0.135, 0.002209322474, 0.0008064027032}; // Placeholder until encoder wires fixed.
    public static final double[] BACK_LEFT_TURNING_GAINS = {0.135, 0.002206896552, 0.0009622068966};
    public static final double[] BACK_RIGHT_TURNING_GAINS = {0.125, 0.002084437086, 0.0007920860927};
    public static final double[][] TURNING_GAINS = {
        FRONT_LEFT_TURNING_GAINS,
        FRONT_RIGHT_TURNING_GAINS,
        BACK_LEFT_TURNING_GAINS,
        BACK_RIGHT_TURNING_GAINS
    };

    public static final LinearSystem<N1, N1, N1> FRONT_LEFT_DRIVING_PLANT = null;
    public static final LinearSystem<N1, N1, N1> FRONT_RIGHT_DRIVING_PLANT = null;
    public static final LinearSystem<N1, N1, N1> BACK_LEFT_DRIVING_PLANT = null;
    public static final LinearSystem<N1, N1, N1> BACK_RIGHT_DRIVING_PLANT = null;
    public static final List<LinearSystem<N1, N1, N1>> DRIVING_PLANTS = new ArrayList<>() {{
        add(FRONT_LEFT_DRIVING_PLANT);
        add(FRONT_RIGHT_DRIVING_PLANT);
        add(BACK_LEFT_DRIVING_PLANT);
        add(BACK_RIGHT_DRIVING_PLANT);
    }};

    public static double[] FRONT_LEFT_TURNING_PID = {1.3, 0.0, 0.0};
    public static double[] FRONT_RIGHT_TURNING_PID = {1.3, 0.0, 0.0};
    public static double[] BACK_RIGHT_TURNING_PID = {1.3, 0.0, 0.0};
    public static double[] BACK_LEFT_TURNING_PID = {1.3, 0.0, 0.0};
    public static double[][] TURNING_PID = {
        FRONT_LEFT_TURNING_PID,
        FRONT_RIGHT_TURNING_PID,
        BACK_LEFT_TURNING_PID,
        BACK_RIGHT_TURNING_PID
    };

    public static double[] FRONT_LEFT_DRIVING_PID = {7.7816, 0.0, 0.031362};
    public static double[] FRONT_RIGHT_DRIVING_PID = {7.8027, 0.0, 0.035489};
    public static double[] BACK_LEFT_DRIVING_PID = {5.1173, 0.0, 0.018606};
    public static double[] BACK_RIGHT_DRIVING_PID = {5.2302, 0.0, 0.029953};
    public static double[][] DRIVING_PID = {
        FRONT_LEFT_DRIVING_PID,
        FRONT_RIGHT_DRIVING_PID,
        BACK_LEFT_DRIVING_PID,
        BACK_RIGHT_DRIVING_PID
    };
}
