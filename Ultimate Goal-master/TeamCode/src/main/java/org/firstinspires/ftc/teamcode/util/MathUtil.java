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


    public static boolean approxEquals(double d1, double d2) {
        if (Double.isInfinite(d1)) {
            // Infinity - infinity is NaN, so we need a special case
            return d1 == d2;
        } else {
            return Math.abs(d1 - d2) < EPSILON;
        }
    }



    private static int sgn(double n) {
        return n < 0 ? -1 : 1;
    }

    public static boolean between(double r1, double r2, double val, double threshold) {
        return val > (Math.min(r1, r2) - threshold) && val < (Math.max(r1, r2) + threshold);
    }
}
