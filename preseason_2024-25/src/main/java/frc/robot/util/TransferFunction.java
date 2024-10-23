package frc.robot.util;

import java.util.LinkedList;
import java.util.Queue;

public class TransferFunction {
    private double[] numerator;
    private double[] denominator;
    private Queue<Double> inputHistory;
    private Queue<Double> outputHistory;

    public TransferFunction(double[] numerator, double[] denominator) {
        this.numerator = numerator;
        this.denominator = denominator;
        this.inputHistory = new LinkedList<>();
        this.outputHistory = new LinkedList<>();
    }

    public double computeOutput(double input) {
        // Add new input to history
        inputHistory.offer(input);
        if (inputHistory.size() > numerator.length) {
            inputHistory.poll();
        }

        // Calculate output based on current and past inputs
        double output = 0.0;

        // Compute numerator
        for (int i = 0; i < numerator.length; i++) {
            if (i < inputHistory.size()) {
                output += numerator[i] * getInputAt(-i);
            }
        }

        // Compute denominator
        double den = 1.0; // Starting with 1 for the denominator
        for (int j = 1; j < denominator.length; j++) {
            if (j <= outputHistory.size()) {
                den += denominator[j] * getOutputAt(-j);
            }
        }

        // Calculate final output
        if (den != 0) {
            output /= den;
        }

        // Add new output to history
        outputHistory.offer(output);
        if (outputHistory.size() > denominator.length - 1) {
            outputHistory.poll();
        }

        return output;
    }

    private double getInputAt(int index) {
        int size = inputHistory.size();
        return (index < 0 && Math.abs(index) <= size) ? (double) inputHistory.toArray()[size + index] : 0.0;
    }

    private double getOutputAt(int index) {
        int size = outputHistory.size();
        return (index < 0 && Math.abs(index) <= size) ? (double) outputHistory.toArray()[size + index] : 0.0;
    }
}
