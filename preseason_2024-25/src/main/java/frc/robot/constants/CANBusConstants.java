package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

public class CANBusConstants {
    // CANIds for the kraken drive motors.
    public static final int FRONT_LEFT_DRIVE = 8;
    public static final int FRONT_RIGHT_DRIVE = 2;
    public static final int BACK_LEFT_DRIVE = 6;
    public static final int BACK_RIGHT_DRIVE = 4;
    public static final List<Integer> DRIVE_IDS = new ArrayList<>() {{
        add(FRONT_LEFT_DRIVE);
        add(FRONT_RIGHT_DRIVE);
        add(BACK_LEFT_DRIVE);
        add(BACK_RIGHT_DRIVE);
    }};

    // CANIds for the neo turning motors.
    public static final int FRONT_LEFT_TURN = 7;
    public static final int FRONT_RIGHT_TURN = 1;
    public static final int BACK_LEFT_TURN = 5;
    public static final int BACK_RIGHT_TURN = 3;
    public static final List<Integer> TURN_IDS = new ArrayList<>() {{
        add(FRONT_LEFT_TURN);
        add(FRONT_RIGHT_TURN);
        add(BACK_LEFT_TURN);
        add(BACK_RIGHT_TURN);
    }};
}
