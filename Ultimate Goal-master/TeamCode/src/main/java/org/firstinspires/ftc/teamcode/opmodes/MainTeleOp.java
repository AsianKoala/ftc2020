package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp
public class MainTeleOp extends Robot {

    Pose startPose = new Pose(0,0,0);
    @Override
    public void init() {
        super.init();
        odometry.setCurrentPosition(startPose);
    }

    @Override
    public void loop() {
        super.loop();

        controlMovement();
    }

    public void controlMovement() {
        double masterScale = 0.5 + ((gamepad1.right_bumper ? 1 : 0) * (1.0-0.5));
        DriveTrain.movementY = -gamepad1.left_stick_y * masterScale;
        DriveTrain.movementX = gamepad1.left_stick_x * masterScale;
        DriveTrain.movementTurn = -gamepad1.right_stick_x * masterScale;
    }
}
