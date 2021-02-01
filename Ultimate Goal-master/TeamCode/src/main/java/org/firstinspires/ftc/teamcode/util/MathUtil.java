package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class MathUtil {
    public static final double EPSILON = 1e-6;

    public static double angleWrap(double angle){
        while (angle<-Math.PI){
            angle += 2*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2*Math.PI;
        }
        return angle;
    }

    public static double clamp(double d) {
        return Math.min(Math.max(d, -1), 1);
    }

    public static double clampAbove(double d, double threshold) {
        return Math.abs(d) < threshold ? Math.copySign(threshold, d) : d;
    }

    public static double cutOffBelow(double d, double threshold) {
        return Math.abs(d) < threshold ? 0 : d;
    }

    public static double powRetainingSign(double d, double power) {
        // In case d is super small, just make it zero
        if (Math.abs(d) < 1e-14) {
            return 0;
        }
        return Math.copySign(Math.pow(Math.abs(d), power), d);
    }

    public static double deadZone(double d, double thresh) {
        return (Math.abs(d) < thresh) ? 0 : d;
    }

    public static Pose relativeOdometryUpdate(Pose fieldPose, Pose robotPoseDelta) {
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
                sineTerm * robotPoseDelta.x - cosTerm * robotPoseDelta.y,
                cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y
        );
        // fieldDeltaX = 1 * dX - 0 * dY
        // fieldDeltaY = 0 * dX + 1 * dY
        // fieldDeltaX = dX, fieldDeltaY = dY

        Pose fieldPoseDelta = new Pose(fieldPositionDelta.rotated(fieldPose.heading), robotPoseDelta.heading);

        return fieldPose.add(fieldPoseDelta);
    }

    public static boolean approxEquals(double d1, double d2) {
        if (Double.isInfinite(d1)) {
            // Infinity - infinity is NaN, so we need a special case
            return d1 == d2;
        } else {
            return Math.abs(d1 - d2) < EPSILON;
        }
    }


    /**
     * Finds the intersection of a line segment and a circle
     * @param circleX x position of the circle
     * @param circleY y position of the circle
     * @param r: radius of the circle
     * @param lineX1 first x position of the line
     * @param lineY1 first y position of the line
     * @param lineX2 second x position of the line
     * @param lineY2 second y position of the line
     * @return an Array of intersections
     */
    public static ArrayList<Point> lineCircleIntersection(double circleX, double circleY, double r,
                                                          double lineX1, double lineY1,
                                                          double lineX2, double lineY2){
        //make sure the points don't exactly line up so the slopes work
        if(Math.abs(lineY1- lineY2) < 0.003){
            lineY1 = lineY2 + 0.003;
        }
        if(Math.abs(lineX1- lineX2) < 0.003){
            lineX1 = lineX2 + 0.003;
        }

        //calculate the slope of the line
        double m1 = (lineY2 - lineY1)/(lineX2-lineX1);

        //the first coefficient in the quadratic
        double quadraticA = 1.0 + pow(m1,2);

        //shift one of the line's points so it is relative to the circle
        double x1 = lineX1-circleX;
        double y1 = lineY1-circleY;


        //the second coefficient in the quadratic
        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1,2) * x1);

        //the third coefficient in the quadratic
        double quadraticC = ((pow(m1,2)*pow(x1,2)) - (2.0*y1*m1*x1) + pow(y1,2)-pow(r,2));


        ArrayList<Point> allPoints = new ArrayList<>();



        //this may give an error so we use a try catch
        try{
            //now solve the quadratic equation given the coefficients
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);

            //we know the line equation so plug into that to get root
            double yRoot1 = m1 * (xRoot1 - x1) + y1;


            //now we can add back in translations
            xRoot1 += circleX;
            yRoot1 += circleY;

            //make sure it was within range of the segment
            double minX = Math.min(lineX1, lineX2);
            double maxX = Math.max(lineX1, lineX2);
            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot1,yRoot1));
            }

            //do the same for the other root
            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);

            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            //now we can add back in translations
            xRoot2 += circleX;
            yRoot2 += circleY;

            //make sure it was within range of the segment
            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new Point(xRoot2,yRoot2));
            }
        }catch(Exception e){
            //if there are no roots
        }
        return allPoints;
    }


    private static int sgn(double n) {
        return n < 0 ? -1 : 1;
    }

    public static boolean between(double r1, double r2, double val, double threshold) {
        return val > (Math.min(r1, r2) - threshold) && val < (Math.max(r1, r2) + threshold);
    }
}
