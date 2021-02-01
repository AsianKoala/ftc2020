package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;

public class BetterOdometry {
//    public static Pose worldPose;
//    public final Pose startPose;
//
//    public static final double TICKS_PER_INCH = 1103.8839;
//
//    public double prevVertical;
//    public double prevHorizontal;
//
//    private OdometrySet odometrySet;
//
//    public BetterOdometry(OdometrySet odometrySet, Pose startPose) {
//        this.odometrySet = odometrySet;
//        worldPose = startPose;
//        this.startPose = startPose;
//        prevVertical = 0;
//        prevHorizontal = 0;
//    }
//
//    public void update(double heading) {
//
//        double deltaHorizontal = odometrySet.getHorizontalTicks() - prevHorizontal;
//        double deltaVertical = odometrySet.getVerticalTicks() - prevVertical;
////        worldPose = MathUtil.relativeOdometryUpdate(worldPose, new Pose(deltaHorizontal/TICKS_PER_INCH, deltaVertical/TICKS_PER_INCH, MathUtil.angleWrap(heading - prevHeading)));
//        // if this doesnt work add angle to robot file
//        // if that doesnt work than xdlmaooooojust die
//        // add startAngle to heading from update xd ?
////        heading = MathUtil.angleWrap(heading + startPose.heading);
//        worldPose.heading = heading;
//
//        worldPose.x += ((deltaVertical * Math.sin(worldPose.heading)) + deltaHorizontal * Math.cos(worldPose.heading))/TICKS_PER_INCH;
//        worldPose.y += ((deltaVertical * Math.cos(worldPose.heading)) - deltaHorizontal * Math.sin(worldPose.heading))/TICKS_PER_INCH;
//        prevHorizontal = odometrySet.getHorizontalTicks();
//        prevVertical = odometrySet.getVerticalTicks();
//    }
//
//
//    @Override
//    public String toString() {
//        return "x: " + worldPose.x + " y: " + worldPose.y + " *: " + Math.toDegrees(worldPose.heading);
//    }
}
