package org.firstinspires.ftc.teamcode.main;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;


public class Odometry {
    public static double PARALLEL_TICKS_PER_INCH = 1111.587;
    public static double LATERAL_TICKS_PER_INCH = 1111.587;
    public static int VELOCITY_READ_TICKS = 5;

    public static int PARALLEL_ENCODER_PORT = 0;
    public static int LATERAL_ENCODER_PORT = 1;

    public static double PARALLEL_Y_POS = -5.728;
    public static double LATERAL_X_POS = -6.944;

    DecompositionSolver forwardSolver;

    int[] prevWheelPositions;
    double prevHeading;

    // External interfaces
    public Pose currentPosition;
    public Pose relativeRobotMovement;

    public Odometry() {
        this(new Pose(0, 0, 0));
    }

    public Odometry(Pose start) {
        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);

        EncoderWheel[] WHEELS = {
                new EncoderWheel(0, PARALLEL_Y_POS, Math.toRadians(180), 0), // parallel
                new EncoderWheel(LATERAL_X_POS, 0, Math.toRadians(90), 1), // lateral
        };

        for (EncoderWheel wheelPosition : WHEELS) {
            double x = Math.cos(wheelPosition.heading);
            double y = Math.sin(wheelPosition.heading);

            inverseMatrix.setEntry(wheelPosition.row, 0, x);
            inverseMatrix.setEntry(wheelPosition.row, 1, y);
            inverseMatrix.setEntry(wheelPosition.row, 2,
                    wheelPosition.x * y - wheelPosition.y * x);
        }
        inverseMatrix.setEntry(2, 2, 1.0);

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();

        if (!forwardSolver.isNonSingular()) {
            throw new IllegalArgumentException("The specified configuration cannot support full localization");
        }

        prevWheelPositions = new int[2]; // Initializes with zeros

        currentPosition = new Pose(start.x, start.y, start.heading);
        relativeRobotMovement = new Pose(0, 0, 0);
    }

    public static double encoderTicksToInches(int ticks, double ticksPerInch) {
        return ticks / ticksPerInch;
    }

    public static int inchesToEncoderTicks(double inches) {
        return (int) (inches * PARALLEL_TICKS_PER_INCH);
    }

    public void update(RevBulkData data, double heading) {
        double[] deltas = new double[] {
                encoderTicksToInches(data.getMotorCurrentPosition(PARALLEL_ENCODER_PORT) - prevWheelPositions[0],
                        PARALLEL_TICKS_PER_INCH),
                encoderTicksToInches(data.getMotorCurrentPosition(LATERAL_ENCODER_PORT) - prevWheelPositions[1],
                        LATERAL_TICKS_PER_INCH),
                MathUtil.angleWrap(heading - prevHeading)
        };
        System.out.println(Arrays.toString(deltas));
        prevWheelPositions[0] = data.getMotorCurrentPosition(PARALLEL_ENCODER_PORT);
        prevHeading = heading;
        prevWheelPositions[1] = data.getMotorCurrentPosition(LATERAL_ENCODER_PORT);
        updateFromRelative(deltas);
    }

    public void updateFromRelative(double[] deltas) {

        RealMatrix m = MatrixUtils.createRealMatrix(new double[][] {deltas});

        RealMatrix rawPoseDelta = forwardSolver.solve(m.transpose());
        Pose robotPoseDelta = new Pose(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        );

        relativeRobotMovement = relativeRobotMovement.add(robotPoseDelta);
        currentPosition = MathUtil.relativeOdometryUpdate(currentPosition, robotPoseDelta);
    }

    public double x() { return currentPosition.x; }
    public double y() { return currentPosition.y; }
    public double h() { return currentPosition.heading; }
    public Pose pose() {
        return new Pose(currentPosition.x, currentPosition.y, currentPosition.heading);
    }


}