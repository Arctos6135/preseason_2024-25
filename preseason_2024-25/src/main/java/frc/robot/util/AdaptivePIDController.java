package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

public class AdaptivePIDController extends PIDController {
    private double lambda; // Forgetting factor
    private double[] theta; // Paremeter vector [kp, ki, kd]
    private double [] P; // Covariance matrix
    private double[] x; // Input vector [error, integral, derivative]
    private double[] y; // Output vector [output]

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a default period of
     * 0.02 seconds.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @param lambda The forgetting factor.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    public AdaptivePIDController(double kp, double ki, double kd, double lambda) {
        super(kp, ki, kd);
        this.lambda = lambda;
        this.theta = new double[]{kp, ki, kd};
        this.P = new double[]{1, 1, 1,}; // Initial covariance matrix
        this.x = new double[3]; // Create vector with three parameters
        this.y = new double[1]; // Create vector with one parameter
    }

    @Override
    /**
     * Returns the next output of the PID controller and updates PID values.
     * 
     * @param currentValue The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double currentValue) {
        // Calculate output using current parameters.
        y[0] = super.calculate(currentValue);

        // Recursive least squares algorithm.
        double[] P_new = new double[3];
        for (int i = 0; i < 3; i++) {
            double K = P[i] * x[i] / (lambda + x[0] * x[0]);
            P_new[i] = P[i] - K * x[i];
            theta[i] += K * (y[0] - theta[0] * x[0] - theta[1] * x[1] - theta[2] * x[2]);
        }

        // Update parameters.
        setP(theta[0]);
        setI(theta[1]);
        setD(theta[2]);
        
        // Reset integral.
        reset();

        // Return computed output
        return y[0];
    }
}
