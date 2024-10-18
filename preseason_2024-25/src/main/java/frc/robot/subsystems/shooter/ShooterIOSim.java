package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim extends ShooterIO {
    private final DCMotorSim leader;
    private final DCMotorSim follower;

    public ShooterIOSim() {
        leader = new DCMotorSim(DCMotor.getAndymark9015(1), 1, 1);
        follower = new DCMotorSim(DCMotor.getAndymark9015(1), 1, 1);
    }

    @Override
    public void setVoltages(double voltage) {
        leader.setInputVoltage(voltage);
        follower.setInputVoltage(voltage);
    }
}
