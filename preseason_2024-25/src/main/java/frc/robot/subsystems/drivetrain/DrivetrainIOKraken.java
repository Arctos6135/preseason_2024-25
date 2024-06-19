package frc.robot.subsystems.drivetrain;



import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.*;



public class DrivetrainIOKraken extends DrivetrainIO {
    // Motor controllers which control the krakens.
    private final TalonFX frontLeftDrive = new TalonFX(CANBusConstants.FRONT_LEFT_DRIVE);
    private final TalonFX frontRightDrive = new TalonFX(CANBusConstants.FORNT_RIGHT_DRIVE);
    private final TalonFX backLeftDrive = new TalonFX(CANBusConstants.BACK_LEFT_DRIVE);
    private final TalonFX backRightDrive = new TalonFX(CANBusConstants.BACK_RIGHT_DRIVE);

    private final AHRS navX = new AHRS();

        public DrivetrainIOKraken(){

        }



}