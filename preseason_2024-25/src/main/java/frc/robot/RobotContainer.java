// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleopDrive;
import frc.robot.constants.OtherConstants;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.MediaSize.Other;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final XboxController driverController = new XboxController(OtherConstants.DRIVER_CONTROLLER);
  private final XboxController operatorController = new XboxController(OtherConstants.OPERATOR_CONTROLLER);

  private final DoubleSupplier driverLeftStickY = () -> MathUtil.applyDeadband(
      driverController.getRawAxis(XboxController.Axis.kLeftY.value),
      OtherConstants.DRIVE_DEADBAND);
    private final DoubleSupplier driverLeftStickX = () -> MathUtil.applyDeadband(
      driverController.getRawAxis(XboxController.Axis.kLeftX.value),
      OtherConstants.DRIVE_DEADBAND);
    private final DoubleSupplier driverRightStickX = () -> MathUtil.applyDeadband(
      driverController.getRawAxis(XboxController.Axis.kRightX.value),
      OtherConstants.DRIVE_DEADBAND);

  private final Swerve drivetrain = new Swerve();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, driverLeftStickY, driverLeftStickX, driverRightStickX));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {}
}
