package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.constants.SwerveConstants;
import frc.robot.util.AdaptivePIDController;

public class SwerveModule {
    private final TalonFX drivingMotor;
    private final CANSparkMax turningMotor;
    private final AnalogEncoder absoluteTurningEncoder;
    private final RelativeEncoder integratedTurningEncoder;
    private final SparkPIDController turningPIDController;
    private final AdaptivePIDController drivingPIDController;
    private final TalonFXConfigurator drivingConfigurator;
    private TalonFXConfiguration drivingConfigs;

    private double chassisAngularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs an MK4i swerve module and configures the driving and turning motor, encoder, and PID controller.
     */
    public SwerveModule(int drivingCANId, int turningCANId, int encoderPort, double chassisAngularOffset) {
        this.drivingMotor = new TalonFX(drivingCANId);
        this.turningMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);
        this.absoluteTurningEncoder = new AnalogEncoder(encoderPort);
        this.drivingConfigurator = drivingMotor.getConfigurator();
        this.integratedTurningEncoder = turningMotor.getEncoder();
        this.turningPIDController = turningMotor.getPIDController();
        this.drivingPIDController = new AdaptivePIDController(1.0, 0.2, 0.5, 0.99);
        this.drivingConfigs = new TalonFXConfiguration();
        this.chassisAngularOffset = chassisAngularOffset;

        configTurningEncoder();
        configTurningMotor();
        configDriveMotor();
    }

    private void resetToAbsolute() {
        double absolutePosition = Math.abs(absoluteTurningEncoder.get() - absoluteTurningEncoder.getPositionOffset());
        integratedTurningEncoder.setPosition(absolutePosition);
    }

    private void configTurningEncoder() {
        absoluteTurningEncoder.setDistancePerRotation(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
    }

    private void configTurningMotor() {
        turningMotor.restoreFactoryDefaults();
        turningMotor.setSmartCurrentLimit(SwerveConstants.TURNING_CURRENT_LIMIT);
        turningMotor.setInverted(true);
        turningMotor.setIdleMode(IdleMode.kBrake);

        turningPIDController.setPositionPIDWrappingEnabled(true);
        turningPIDController.setPositionPIDWrappingMinInput(0);
        turningPIDController.setPositionPIDWrappingMaxInput(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);

        integratedTurningEncoder.setPositionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
        integratedTurningEncoder.setVelocityConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR);
        resetToAbsolute();

        turningMotor.burnFlash();
    }

    private void configDriveMotor() {
        drivingConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        drivingConfigs.CurrentLimits.StatorCurrentLimit = 40;
        drivingConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        drivingConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        drivingConfigurator.apply(drivingConfigs);

        desiredState.angle = new Rotation2d(absoluteTurningEncoder.getDistance());
        drivingMotor.setPosition(0);
    }

    /**
     * Gets the distance in meters.
     */
    public double getDistance() {
        return drivingMotor.getPosition().getValueAsDouble() * SwerveConstants.DRIVING_ENCODER_POSITION_FACTOR;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(absoluteTurningEncoder.getDistance());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDistance(), getAngle());
    }

    public void setState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState,
            new Rotation2d(integratedTurningEncoder.getPosition())
        );

        drivingPIDController.setSetpoint(optimizedDesiredState.speedMetersPerSecond / SwerveConstants.DRIVE_ENCODER_POSITION_FACTOR);
        drivingMotor.setControl(new VoltageOut(drivingPIDController.calculate(drivingMotor.get())));
        turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);
    }
}