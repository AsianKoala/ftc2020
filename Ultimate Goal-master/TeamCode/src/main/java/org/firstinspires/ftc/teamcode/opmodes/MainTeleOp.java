package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp(name="main teleop")
public class MainTeleOp extends Robot {
    Point anglePoint;
    boolean headingControlled = false;

    @Override
    public void init() {
        super.init();
        odometry.setStart(new Pose(0, 0, 0));
        anglePoint = new Point(0,0);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        controlMovement();
        controlAnglePoint();

        if(gamepad1.x) {
            odometry.setGlobalPosition(new Point(0, 0));
        }
    }

    public void controlMovement() {
        double masterScale = 0.5 + ((gamepad1.right_bumper ? 1 : 0) * 0.5);
        DriveTrain.movementY = -gamepad1.left_stick_y * masterScale;
        DriveTrain.movementX = gamepad1.left_stick_x * masterScale;
        DriveTrain.movementTurn = -gamepad1.right_stick_x * masterScale;
    }

    @SuppressLint("DefaultLocale")
    public void controlAnglePoint() {
        if(gamepad1.left_bumper) {
            anglePoint.x += 6;
        }
        if(gamepad1.left_trigger > 0.7) {
            anglePoint.x -= 6;
        }
        if(gamepad1.right_bumper) {
            anglePoint.y += 6;
        }
        if(gamepad1.right_trigger > 0.7) {
            anglePoint.y -= 6;
        }

        if(gamepad1.y) {
            headingControlled = !headingControlled;
        }

        if(headingControlled) {
            PPController.movementResult result = PPController.pointPointTurn(anglePoint, 0.6, Math.toRadians(30));
            telemetry.addLine(String.format("movementR: %.2f", Math.toDegrees(result.turnDelta_rad)));
            telemetry.addLine("anglePoint: " + anglePoint.toString());
        }
    }
}
