package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.util.Pose;

public class BetterOdometry {
    public static Pose worldPose;

    public static final double TICKS_PER_INCH = 1103.8839;

    public double prevVertical;
    public double prevHorizontal;

    private OdometrySet odometrySet;

    public BetterOdometry(OdometrySet odometrySet) {
        this.odometrySet = odometrySet;
        worldPose = new Pose(0, 0, 0);
        prevVertical = 0;
        prevHorizontal = 0;
    }

    public void update(double heading) {
        double deltaHorizontal = odometrySet.getHorizontalTicks() - prevHorizontal;
        double deltaVertical = odometrySet.getVerticalTicks() - prevVertical;

        worldPose.heading = heading;

        worldPose.x += ((deltaVertical * Math.sin(worldPose.heading)) + deltaHorizontal * Math.cos(worldPose.heading))/TICKS_PER_INCH;
        worldPose.y += ((deltaVertical * Math.cos(worldPose.heading)) - deltaHorizontal * Math.sin(worldPose.heading))/TICKS_PER_INCH;

        prevHorizontal = odometrySet.getHorizontalTicks();
        prevVertical = odometrySet.getVerticalTicks();
    }

    public void setWorldPose(Pose pose) {
        worldPose = pose;
    }

    @Override
    public String toString() {
        return "better odometry: x: " + worldPose.x + " y: " + worldPose.y + " *: " + Math.toDegrees(worldPose.heading);
    }
}
