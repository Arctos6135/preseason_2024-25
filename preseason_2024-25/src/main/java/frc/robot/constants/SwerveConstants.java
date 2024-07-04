package frc.robot.constants;

import java.util.ArrayList;
import java.util.Arrays;
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

    public static final int FRONT_LEFT_ENCODER_PORT = 1;
    public static final int FRONT_RIGHT_ENCODER_PORT = 2;
    public static final int BACK_LEFT_ENCODER_PORT = 3;
    public static final int BACK_RIGHT_ENCODER_PORT = 4;
    public static final List<Integer> ENCODER_PORTS = new ArrayList<>() {{
        add(FRONT_LEFT_ENCODER_PORT);
        add(FRONT_RIGHT_ENCODER_PORT);
        add(BACK_LEFT_ENCODER_PORT);
        add(BACK_RIGHT_ENCODER_PORT);
    }};
    
    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_ANGULAR_OFFSET = Math.PI;
    public static final double BACK_RIGHT_ANGULAR_OFFSET = Math.PI / 2;
    public static final List<Double> ANGULAR_OFFSETS = new ArrayList<>() {{
        add(FRONT_LEFT_ANGULAR_OFFSET);
        add(FRONT_RIGHT_ANGULAR_OFFSET);
        add(BACK_LEFT_ANGULAR_OFFSET);
        add(BACK_RIGHT_ANGULAR_OFFSET);
    }};

    // This of course assumes we manage to actually get the COG in the middle
    public static final double DISTANCE_TO_CENTER = 12.5;

    public static final double DRIVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(Math.PI * 4.0);

    public static final int TURNING_CURRENT_LIMIT = 40;
    public static final int DRIVING_CURRENT_LIMIT = 60;

    // In radians / radians per second.
    public static final double TURNING_ENCODER_POSITION_FACTOR = 2 * Math.PI;
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = 2 * Math.PI / 60.0;

    // In meters.
    public static final double DRIVING_ENCODER_POSITION_FACTOR = DRIVE_WHEEL_CIRCUMFERENCE;
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = DRIVING_ENCODER_POSITION_FACTOR / 60.0;

    public static final double MAX_SPEED = 4;
    public static final double MAX_ANGULAR_VELOCITY = 3;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double DRIVE_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER * Math.PI);

    // Plants for feedforward and simulation purposes.
    public static final LinearSystem<N1, N1, N1> FRONT_LEFT_TURNING_PLANT = null;
    public static final LinearSystem<N1, N1, N1> FRONT_RIGHT_TURNING_PLANT = null;
    public static final LinearSystem<N1, N1, N1> BACK_LEFT_TURNING_PLANT = null;
    public static final LinearSystem<N1, N1, N1> BACK_RIGHT_TURNING_PLANT = null;
    public static final List<LinearSystem<N1, N1, N1>> TURNING_PLANTS = new ArrayList<>() {{
        add(FRONT_LEFT_TURNING_PLANT);
        add(FRONT_RIGHT_TURNING_PLANT);
        add(BACK_LEFT_TURNING_PLANT);
        add(BACK_RIGHT_TURNING_PLANT);
    }};

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

    public static final List<Double> FRONT_LEFT_TURNING_PID = new ArrayList<>() {{
        Arrays.asList(1.0, 0.2, 0.2); // First value is P, second is I, third is D
    }};
    public static final List<Double> FRONT_RIGHT_TURNING_PID = new ArrayList<>() {{
        Arrays.asList(1.0, 0.2, 0.2);
    }};
    public static final List<Double> BACK_LEFT_TURNING_PID = new ArrayList<>() {{
        Arrays.asList(1.0, 0.2, 0.2);
    }};
    public static final List<Double> BACK_RIGHT_TURNING_PID = new ArrayList<>() {{
        Arrays.asList(1.0, 0.2, 0.2);
    }};
    public static final List<List<Double>> TURNING_PID = new ArrayList<>() {{
        add(FRONT_LEFT_TURNING_PID);
        add(FRONT_RIGHT_TURNING_PID);
        add(BACK_LEFT_TURNING_PID);
        add(BACK_RIGHT_TURNING_PID);
    }};

    public static final List<Double> FRONT_LEFT_DRIVING_PID = new ArrayList<>() {{
        Arrays.asList(1.0, 0.2, 0.2);
    }};
    public static final List<Double> FRONT_RIGHT_DRIVING_PID = new ArrayList<>() {{
        Arrays.asList(1.0, 0.2, 0.2);
    }};
    public static final List<Double> BACK_LEFT_DRIVING_PID = new ArrayList<>() {{
        Arrays.asList(1.0, 0.2, 0.2);
    }};
    public static final List<Double> BACK_RIGHT_DRIVING_PID = new ArrayList<>() {{
        Arrays.asList(1.0, 0.2, 0.2);
    }};
    public static final List<List<Double>> DRIVING_PID = new ArrayList<>() {{
        add(FRONT_LEFT_DRIVING_PID);
        add(FRONT_RIGHT_DRIVING_PID);
        add(BACK_LEFT_DRIVING_PID);
        add(BACK_RIGHT_DRIVING_PID);
    }};
}
