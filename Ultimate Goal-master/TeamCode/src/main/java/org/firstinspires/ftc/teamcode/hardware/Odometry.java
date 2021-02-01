package org.firstinspires.ftc.teamcode.hardware;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.opmodes.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.Arrays;


public class Odometry {
    public static double currX;
    public static double currY;
    public static double currHeading;
    public static Pose currPose;
    // 8192 ticks per revolution
    // wheels are 60mm, or 2.3622 inches diameter
    // 2.3622 * pi = 7.42107016631 circumference
    // 8192 / 7.42107016631 = ticks per inch
    public static final double TICKS_PER_INCH = 1103.8839;
    public static final double PARALLEL_Y_POS = 0; // -5.728
    public static final double LATERAL_X_POS = 0; // -6.944

    private DecompositionSolver forwardSolver;

    public double startHeading;
    private int prevVertical;
    private int prevHorizontal;
    private double prevHeading;

    private Pose currentPosition;

    private OdometrySet odometrySet;

    private Robot opMode;
//    public Odometry(OdometrySet odometrySet) {
//        this(new Pose(0, 0, 0), odometrySet);
//    }

    public Odometry(Pose start, OdometrySet odometrySet, Robot opMode) {
        this.opMode = opMode;
        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);

        EncoderWheel[] WHEELS = {
                new EncoderWheel(0, PARALLEL_Y_POS, Math.toRadians(0), 0), // parallel
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

        startHeading = start.heading;
        prevHorizontal = 0;
        prevVertical = 0;
        prevHeading = startHeading;
        this.odometrySet = odometrySet;

        currentPosition = start;
        updateValues();
    }

    public static double encoderTicksToInches(int ticks, double ticksPerInch) {
        return ticks / ticksPerInch;
    }

    public void update(double heading) {
        double[] deltas = new double[] {
                encoderTicksToInches(odometrySet.getHorizontalTicks() - prevHorizontal,
                        TICKS_PER_INCH),
                encoderTicksToInches(odometrySet.getVerticalTicks() - prevVertical,
                        TICKS_PER_INCH),
                MathUtil.angleWrap(heading - prevHeading)
        };
        System.out.println(Arrays.toString(deltas));
        opMode.telemetry.addLine("odom deltas: " + Arrays.toString(deltas));
        prevVertical = odometrySet.getVerticalTicks();
        prevHorizontal = odometrySet.getHorizontalTicks();
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

        opMode.telemetry.addLine("raw pose deltas: " + robotPoseDelta.toString());
//        double deltaX = odometrySet.getHorizontalTicks() - prev

        double newHeading = MathUtil.angleWrap(currentPosition.heading + robotPoseDelta.heading);
        double fieldDeltaX = (Math.cos(newHeading) * robotPoseDelta.y) + (Math.sin(newHeading) * robotPoseDelta.x);
        double fieldDeltaY = (Math.sin(newHeading) * robotPoseDelta.y) - (Math.cos(newHeading) * robotPoseDelta.x);
        Pose addPose = new Pose(fieldDeltaX, fieldDeltaY, robotPoseDelta.heading);

        opMode.telemetry.addLine("field deltas: " + addPose.toString());
        currentPosition.add(new Pose(fieldDeltaX, fieldDeltaY,robotPoseDelta.heading));
        updateValues();
    }

    public void setCurrentPosition(Pose startPosition) {
        currentPosition = startPosition;
        updateValues();
    }

    private void updateValues() {
        currX = currentPosition.x;
        currY = currentPosition.y;
        currHeading = currentPosition.heading;
        currPose = currentPosition;
    }


    public String toString() {
        return "curr odom readings: " + currentPosition.toString();
    }
}