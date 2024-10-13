// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleopDrive;
import frc.robot.commands.characterization.StepVoltageRoutine;
import frc.robot.commands.shooter.Intake;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.utility.resetAbsoluteEncoders;
import frc.robot.constants.OtherConstants;
import frc.robot.constants.PositionConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIOSim;
import frc.robot.subsystems.swerve.SwerveIOSparkMax;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("autoChooser");
  public LoggedDashboardChooser<Pose2d> positionChooser = new LoggedDashboardChooser<>("positionChooser");

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
      OtherConstants.DRIVE_DEADBAND * 2);

  private final Swerve drivetrain;
  private final Shooter shooter;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Robot.isReal()) {
    drivetrain = new Swerve(new SwerveIOSparkMax());
    shooter = new Shooter(new ShooterIOSparkMax());
    }
    else {
      drivetrain = new Swerve(new SwerveIOSim());
      shooter = new Shooter(new ShooterIOSim());
    }


    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, driverLeftStickY, driverLeftStickX, driverRightStickX));

    autoChooser.addDefaultOption("Source Score and Leave", new PathPlannerAuto("Source Score and Leave"));
    autoChooser.addOption("Amp Score and Leave", new PathPlannerAuto("Amp Score and Leave"));
    autoChooser.addOption("Stage Score and Leave", new PathPlannerAuto("Stage Score and Leave"));

    NamedCommands.registerCommand("shoot", Shoot.shoot(shooter));

    positionChooser.addDefaultOption("RED_AMP", PositionConstants.RED_AMP);
    positionChooser.addOption("RED_STAGE", PositionConstants.RED_STAGE);
    positionChooser.addOption("RED_SOURCE", PositionConstants.RED_SOURCE);

    positionChooser.addOption("BLUE_AMP", PositionConstants.BLUE_AMP);
    positionChooser.addOption("BLUE_STAGE", PositionConstants.BLUE_STAGE);
    positionChooser.addOption("BLUE_SOURCE", PositionConstants.BLUE_SOURCE);

    // Configure the trigger bindings
    configureBindings();
    configSmartDashboard();
  }

  private void configSmartDashboard() {
    SmartDashboard.putData("resetAbsoluteEncoders", new resetAbsoluteEncoders(drivetrain));
    SmartDashboard.putData("StepVoltageRoutine", new StepVoltageRoutine(drivetrain));
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
  private void configureBindings() {
    Trigger operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    Trigger operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    Trigger operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    Trigger operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);

    operatorA.onTrue(Shoot.shoot(shooter));
    operatorB.whileTrue(new Intake(shooter));

  }

  public void startMatch() {
    drivetrain.resetPose(positionChooser.get());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
