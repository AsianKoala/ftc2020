package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;


@Autonomous
public class TestAuto extends Auto {


    public enum progStages {
        driveForward,
        turn,
        goToPosition,
        stop
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        setStage(progStages.goToPosition.ordinal());
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void MainStateMachine() {
        super.MainStateMachine();
        if(progStage == progStages.driveForward.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            DriveTrain.movementY = 0.5;
            if(Math.abs(startStageY - Odometry.currentPosition.y) > 5) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progStage == progStages.turn.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            DriveTrain.movementTurn = 0.3;
            if(Math.abs(Math.toDegrees(MathUtil.angleWrap(startStageHeading - Odometry.currentPosition.heading))) > 45) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progStage == progStages.goToPosition.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }


            PPController.goToPosition(36, 36, 0.75, Math.toRadians(90), 0.5, Math.toRadians(30), 0, false);
            if(Odometry.currentPosition.distanceBetween(new Pose(-24,24,0)) < 1)
                DriveTrain.stopMovement();
        }

        if(progStage == progStages.stop.ordinal()) {
            requestOpModeStop();
        }
    }
}
