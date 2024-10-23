package frc.robot.util;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class StateSpaceModel {
    private Matrix<N2, N2> A;
    private Matrix<N2, N1> B;
    private Matrix<N1, N2> C;
    private Matrix<N1, N1> D;

    private Matrix<N2, N1> stateVector;

    double[][] initialState = {{0.0}, {0.0}};

    public StateSpaceModel(double[][] systemMatrix, double[][] inputMatrix, double[] outputMatrix, double[] feedthroughMatrix) {
        this.A = new Matrix<N2, N2>(new SimpleMatrix(systemMatrix));
        this.B = new Matrix<N2, N1>(new SimpleMatrix(inputMatrix));
        this.C = new Matrix<N1, N2>(new SimpleMatrix(outputMatrix).transpose());
        this.D = new Matrix<N1, N1>(new SimpleMatrix(feedthroughMatrix));

        stateVector = new Matrix<N2, N1>(new SimpleMatrix(initialState));
    }

    public void update(double input) {
        double state0 = A.get(0,0) * stateVector.get(0, 0) +  A.get(0, 1) * stateVector.get(1, 0);
        double state1 = A.get(1,0) * stateVector.get(0, 0) +  A.get(1, 1) * stateVector.get(1, 0);

        stateVector.set(0, 0, state0);
        stateVector.set(1, 0, state1);

        stateVector = stateVector.plus(B.times(input));
    }

    public double getOutput() {
        return stateVector.get(0,0) * C.get(0, 0) + stateVector.get(1, 0) * C.get(0, 1);
    }
}