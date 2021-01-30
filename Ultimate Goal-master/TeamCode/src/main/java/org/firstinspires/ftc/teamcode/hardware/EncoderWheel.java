package org.firstinspires.ftc.teamcode.hardware;

public class EncoderWheel {
    public double x, y, heading;
    public int row;
    public EncoderWheel(double x, double y, double heading, int row) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.row = row;
    }
}
