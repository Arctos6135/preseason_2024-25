package frc.robot.util;

public class SimMotor {
    private double[] den;
    private double[] num;

    private double lastVelocity = 0.0;
    private double velocity;
    private double position;

    private double voltage = 0.0;

    private StateSpaceModel stateSpaceModel;

    public SimMotor(double[][] A, double[][] B, double[] C, double[] D) {
        this.stateSpaceModel = new StateSpaceModel(A, B, C, D);
    }

    public void update(double dt) {
        stateSpaceModel.update(voltage);
        velocity = stateSpaceModel.getOutput();

        position += ((velocity + lastVelocity) / 2) * dt;
        lastVelocity = velocity;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getPosition() {
        return position;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
