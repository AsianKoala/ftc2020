package org.firstinspires.ftc.teamcode.main;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.Arrays;


public class Odometry {
    // 8192 ticks per revolution
    // wheels are 60mm, or 2.3622 inches diameter
    // 2.3622 * pi = 7.42107016631 circumference
    // 8192 / 7.42107016631 = ticks per inch
    // 1103.88391653 ticks per inch
    public static final double TICKS_PER_INCH = 1103.8839;
    public static final double PARALLEL_Y_POS = -5.728;
    public static final double LATERAL_X_POS = -6.944;

    private DecompositionSolver forwardSolver;

    private int prevParallel;
    private int prevLateral;
    private double prevHeading;

    private Pose currentPosition;
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

        prevLateral = 0;
        prevParallel = 0;

        currentPosition = new Pose(start.x, start.y, start.heading);
        relativeRobotMovement = new Pose(0, 0, 0);
    }

    public static double encoderTicksToInches(int ticks, double ticksPerInch) {
        return ticks / ticksPerInch;
    }

    public void update(int parallel_ticks, int lateral_ticks, double heading) {
        double[] deltas = new double[] {
                encoderTicksToInches(parallel_ticks - prevParallel,
                        TICKS_PER_INCH),
                encoderTicksToInches(lateral_ticks - prevLateral,
                        TICKS_PER_INCH),
                MathUtil.angleWrap(heading - prevHeading)
        };
        System.out.println(Arrays.toString(deltas));
        prevParallel = parallel_ticks;
        prevLateral = lateral_ticks;
        prevHeading = heading;
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
    public double heading() { return currentPosition.heading; }
    public Pose pose() {
        return currentPosition;
    }

    public String toString() {
        return pose().toString();
    }
}