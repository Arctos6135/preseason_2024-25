package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
    public static final double FRONT_LEFT_POSITION_OFFSET = 0.993;
    public static final double FRONT_RIGHT_POSITION_OFFSET = 0.397;
    public static final double BACK_LEFT_POSITION_OFFSET = 0.521;
    public static final double BACK_RIGHT_POSITION_OFFSET = 0.155;
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
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = (DRIVE_WHEEL_CIRCUMFERENCE / DRIVING_GEARING_RATIO);

    public static final double MAX_SPEED = 3.0;
    public static final double MAX_ANGULAR_VELOCITY = 2.5;

    // Gains for feedforward and simulation purposes.
    public static final double[] STEADY_STATE_GAINS = {
        0.21599,
    };

    public static double[] FRONT_LEFT_TURNING_PID = {2.3, 0.0, 0.0};
    public static double[] FRONT_RIGHT_TURNING_PID = {2.3, 0.0, 0.0};
    public static double[] BACK_RIGHT_TURNING_PID = {2.3, 0.0, 0.0};
    public static double[] BACK_LEFT_TURNING_PID = {2.3, 0.0, 0.0};
    public static double[][] TURNING_PID = {
        FRONT_LEFT_TURNING_PID,
        FRONT_RIGHT_TURNING_PID,
        BACK_LEFT_TURNING_PID,
        BACK_RIGHT_TURNING_PID
    };

    public static double[] FRONT_LEFT_DRIVING_PID = {2.0, 0.0, 0.0};
    public static double[] FRONT_RIGHT_DRIVING_PID = {2.0, 0.0, 0.0};
    public static double[] BACK_LEFT_DRIVING_PID = {2.0, 0.0 ,0.0};
    public static double[] BACK_RIGHT_DRIVING_PID = {2.0, 0.0, 0.0};
    public static double[][] DRIVING_PID = {
        FRONT_LEFT_DRIVING_PID,
        FRONT_RIGHT_DRIVING_PID,
        BACK_LEFT_DRIVING_PID,
        BACK_RIGHT_DRIVING_PID
    };

    public static double[] FRONT_LEFT_DRIVING_FEEDFORWARD = {0, 2.3215, 0.04485};
    public static double[] FRONT_RIGHT_DRIVING_FEEDFORWARD = {0, 2.3447,  0.06620};
    public static double[] BACK_LEFT_DRIVING_FEEDFORWARD = {0, 2.2687, 0.07688};
    public static double[] BACK_RIGHT_DRIVING_FEEDFORWARD = {0, 2.3215, 0.05213};
    public static double[][] DRIVING_FEEDFORWARDS = {
        FRONT_LEFT_DRIVING_FEEDFORWARD,
        FRONT_RIGHT_DRIVING_FEEDFORWARD,
        BACK_LEFT_DRIVING_FEEDFORWARD,
        BACK_RIGHT_DRIVING_FEEDFORWARD
    };


    public static List<LinearSystem<N2, N1, N2>> DRIVING_MOTOR_LINEAR_SYSTEMS = new ArrayList<>() {{
        add(LinearSystemId.createDCMotorSystem(FRONT_LEFT_DRIVING_FEEDFORWARD[1], FRONT_LEFT_DRIVING_FEEDFORWARD[2]));
        add(LinearSystemId.createDCMotorSystem(FRONT_RIGHT_DRIVING_FEEDFORWARD[1], FRONT_LEFT_DRIVING_FEEDFORWARD[2]));
        add(LinearSystemId.createDCMotorSystem(BACK_LEFT_DRIVING_FEEDFORWARD[1], FRONT_LEFT_DRIVING_FEEDFORWARD[2]));
        add(LinearSystemId.createDCMotorSystem(BACK_RIGHT_DRIVING_FEEDFORWARD[1], FRONT_LEFT_DRIVING_FEEDFORWARD[2]));
    }};

    public static List<LinearSystem<N2, N1, N2>> TURNING_MOTOR_LINEAR_SYSTEMS = new ArrayList<>() {{
        add(LinearSystemId.createDCMotorSystem(0.062527, 0.0002841));
        add(LinearSystemId.createDCMotorSystem(0.062527, 0.0002841));
        add(LinearSystemId.createDCMotorSystem(0.062527, 0.0002841));
        add(LinearSystemId.createDCMotorSystem(0.062527, 0.00028416));
    }};



    public static HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(0.2, 0.0, 0.0),
        new PIDConstants(0.2, 0.0, 0.0),
        SwerveConstants.MAX_SPEED,
        SwerveConstants.DISTANCE_TO_CENTER,
        new ReplanningConfig()
    );
}
