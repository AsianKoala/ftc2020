package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.teamcode.opmodes.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.jetbrains.annotations.NotNull;


public class Odometry {
    // 8192 ticks per revolution
    // wheels are 60mm, or 2.3622 inches diameter
    // 2.3622 * pi = 7.42107016631 circumference
    // 8192 / 7.42107016631 = ticks per inch
    public static final double TICKS_PER_INCH = 1103.8839;

    private int prevVertical;
    private int prevHorizontal;
    private double prevHeading;

    public static double startHeading;
    public static Pose currentPosition;

    private final OdometrySet odometrySet;


    public Odometry(Pose start, OdometrySet odometrySet) {
        startHeading = start.heading;
        prevHorizontal = 0;
        prevVertical = 0;
        prevHeading = startHeading;
        this.odometrySet = odometrySet;

        currentPosition = start;
    }

    public void setStart(Pose start) {
        startHeading = start.heading;
        prevHeading = startHeading;
        currentPosition = start;
    }

    public void update(double heading) {
        double deltaY = (odometrySet.getVerticalTicks() - prevVertical) / TICKS_PER_INCH;
        double deltaX = (odometrySet.getHorizontalTicks() - prevHorizontal) / TICKS_PER_INCH;
        double deltaAngle = MathUtil.angleWrap(heading - prevHeading);

        double newHeading = MathUtil.angleWrap(currentPosition.heading + deltaAngle);
        currentPosition.x += - (Math.cos(newHeading) * deltaY) + (Math.sin(newHeading) * deltaX);
        currentPosition.y += - (Math.sin(newHeading) * deltaY) - (Math.cos(newHeading) * deltaX);
        currentPosition.heading = newHeading;

        prevHorizontal = odometrySet.getHorizontalTicks();
        prevVertical = odometrySet.getVerticalTicks();
        prevHeading = currentPosition.heading;
    }


    @NotNull
    public String toString() {
        return "curr odom readings: " + currentPosition.toString();
    }
}