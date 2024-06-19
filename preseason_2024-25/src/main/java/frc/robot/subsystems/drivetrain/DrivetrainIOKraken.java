package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix5.*;

import frc.robot.constants;

public class DrivetrainIOKraken extends DrivetrainIO {
    // Motor controllers which control the krakens.
    private final TalonFX frontLeftDrive = new TalonFX(Krakenconstants.frontLeftDrive);
    private final TalonFX frontRightDrive = new TalonFX(constants.frontRightDrive);
    private final TalonFX backLeftDrive = new TalonFX(constants.backLeftDrive);
    private final TalonFX backRightDrive = new TalonFX(constants.backRightDrive);

    // Relative encoders to know position and velocity.
    private final RelativeEncoder frontLeftDriveEncoder;
    private final RelativeEncoder frontRightDriveEncoder;
    private final RelativeEncoder backLeftDriveEncoder;
    private final RelativeEncoder backRightDriveEncoder;



}