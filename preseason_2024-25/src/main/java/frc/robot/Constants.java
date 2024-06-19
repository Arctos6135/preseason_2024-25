// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CANBusConstants {
    // CANIds for the kraken drive motors.
    public static final int leftFrontDrive = 1;
    public static final int rightFrontDrive = 2;
    public static final int leftBackDrive = 3;
    public static final int rightBackDrive = 4;

    // CANIds for the neo turning motors.
    public static final int leftFrontTurn = 5;
    public static final int rightFrontTurn = 6;
    public static final int leftBackTurn = 7;
    public static final int rightBackTurn = 8;

    // CANIds for absolute encoders on turning motors.
    public static final int leftFrontEncoder = 9;
    public static final int rightFrontEncoder = 10;
    public static final int leftBackEncoder = 11;
    public static final int rightBackEncoder = 12;

    // NavX (gyro).
    public static final int NavX = 13;
  }
}
