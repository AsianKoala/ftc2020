//package org.firstinspires.ftc.teamcode.movement;
//
//import org.firstinspires.ftc.teamcode.util.MathUtil;
//import org.firstinspires.ftc.teamcode.util.Point;
//import org.firstinspires.ftc.teamcode.util.Pose;
//import org.jetbrains.annotations.NotNull;
//
//
//public class Odometry {
//    // 8192 ticks per revolution
//    // wheels are 60mm, or 2.3622 inches diameter
//    // 2.3622 * pi = 7.42107016631 circumference
//    // 8192 / 7.42107016631 = ticks per inch
//    public static final double TICKS_PER_INCH = 1103.8839;
//
//    private int prevVertical;
//    private int prevHorizontal;
//    private double prevHeading;
//
//    public static double startHeading;
//    public static Pose currentPosition;
//
//    private final OdometrySet odometrySet;
//
//
//    public Odometry(Pose start, OdometrySet odometrySet) {
//        startHeading = start.heading;
//        prevHorizontal = 0;
//        prevVertical = 0;
//        prevHeading = startHeading;
//        this.odometrySet = odometrySet;
//
//        currentPosition = start;
//    }
//
//    public void setStart(Pose start) {
//        startHeading = start.heading;
//        prevHeading = startHeading;
//        currentPosition = start;
//    }
//
//    // very very dangerous
//    public void setGlobalPosition(Point newPosition) {
//        currentPosition = new Pose(newPosition, currentPosition.heading);
//    }
//
//    public void update(double heading) {
//        double deltaY = (odometrySet.getVerticalTicks() - prevVertical) / TICKS_PER_INCH;
//        double deltaX = (odometrySet.getHorizontalTicks() - prevHorizontal) / TICKS_PER_INCH;
//        double deltaAngle = MathUtil.angleWrap(heading - prevHeading);
//
//        double newHeading = MathUtil.angleWrap(currentPosition.heading + deltaAngle);
//        currentPosition.x += - (Math.cos(newHeading) * deltaY) + (Math.sin(newHeading) * deltaX);
//        currentPosition.y += - (Math.sin(newHeading) * deltaY) - (Math.cos(newHeading) * deltaX);
//        currentPosition.heading = newHeading;
//
//        prevHorizontal = odometrySet.getHorizontalTicks();
//        prevVertical = odometrySet.getVerticalTicks();
//        prevHeading = currentPosition.heading;
//    }
//
//
//    @NotNull
//    public String toString() {
//        return "curr odom readings: " + currentPosition.toString();
//    }
//}




package org.firstinspires.ftc.teamcode.movement;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.util.*;
import org.jetbrains.annotations.NotNull;


import static org.firstinspires.ftc.teamcode.util.MathUtil.angleWrap;
import static org.firstinspires.ftc.teamcode.util.MathUtil.approxEquals;

public class Odometry {

        private int prevVertical;
        private int prevHorizontal;
        private double prevHeading;
        public static double startHeading;
        public static Pose currentPosition;
        private final OdometrySet odometrySet;

        public static final double TICKS_PER_INCH = 1103.8839;


//    public Odometry(Pose start, OdometrySet odometrySet) {
//        startHeading = start.heading;
//        prevHorizontal = 0;
//        prevVertical = 0;
//        prevHeading = startHeading;
//        this.odometrySet = odometrySet;
//
//        currentPosition = start;
//    }


        public static double PARALLEL_TICKS_PER_INCH = 1103.8839;
        public static double LATERAL_TICKS_PER_INCH = 1103.8839;

        public static double PARALLEL_X = 6.75;
        public static double PARALLEL_Y = 4.19;

        public static double PERPENDICULAR_X = 7.13;
        public static double PERPENDICULAR_Y = -4.12;



        public static double PARALLEL_Y_POS = 4.19;
        public static double LATERAL_X_POS = 7.13;

        DecompositionSolver forwardSolver;



        public Odometry(Pose start, OdometrySet odometrySet) {
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

            startHeading = start.heading;
            prevHorizontal = 0;
            prevVertical = 0;
            prevHeading = startHeading;
            this.odometrySet = odometrySet;
        }


        public void update(double heading) {
            double deltaY = (odometrySet.getVerticalTicks() - prevVertical) / TICKS_PER_INCH;
            double deltaX = (odometrySet.getHorizontalTicks() - prevHorizontal) / TICKS_PER_INCH;
            double deltaAngle = MathUtil.angleWrap(heading - prevHeading);
            double[] deltas = new double[] {deltaY, deltaX, deltaAngle};

            prevHorizontal = odometrySet.getHorizontalTicks();
            prevVertical = odometrySet.getVerticalTicks();
            prevHeading = currentPosition.heading;

            RealMatrix m = MatrixUtils.createRealMatrix(new double[][] {deltas});

            RealMatrix rawPoseDelta = forwardSolver.solve(m.transpose());
            Pose robotPoseDelta = new Pose(
                    rawPoseDelta.getEntry(0, 0),
                    rawPoseDelta.getEntry(1, 0),
                    rawPoseDelta.getEntry(2, 0)
            );


            double dtheta = robotPoseDelta.heading;
            double sineTerm, cosTerm;

            if (approxEquals(dtheta, 0)) {
                sineTerm = 1.0 - dtheta * dtheta / 6.0;
                cosTerm = dtheta / 2.0;
            } else {
                sineTerm = Math.sin(dtheta) / dtheta;
                cosTerm = (1 - Math.cos(dtheta)) / dtheta;
            }

            Point fieldPositionDelta = new Point(
                    (sineTerm * robotPoseDelta.x) - (cosTerm * robotPoseDelta.y),
                    (cosTerm * robotPoseDelta.x) + (sineTerm * robotPoseDelta.y)
            );

            Pose fieldPoseDelta = new Pose(fieldPositionDelta.rotated(currentPosition.heading), robotPoseDelta.heading);

            currentPosition.x += fieldPoseDelta.x;
            currentPosition.y += fieldPoseDelta.y;
            currentPosition.heading = angleWrap(currentPosition.heading + fieldPoseDelta.heading);
        }



        public void setStart(Pose start) {
        startHeading = start.heading;
        prevHeading = startHeading;
        currentPosition = start;
        }


        // very very dangerous
        public void setGlobalPosition(Point newPosition) {
        currentPosition = new Pose(newPosition, currentPosition.heading);
        }

        @NotNull
        public String toString() {
        return "curr odom readings: " + currentPosition.toString();
        }

}


class EncoderWheel {
    public double x, y, heading;
    public int row; // Row in matrix

    EncoderWheel(double x, double y, double heading, int row) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.row = row;
    }
}