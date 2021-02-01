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
//        if(progStage == progStages.driveForward.ordinal()) {
//            if(stageFinished) {
//                initProgVars();
//            }
//
//            DriveTrain.movementY = 0.5;
//            telemetry.addLine("Currently on stage: " + progStages.driveForward.name());
//            telemetry.addLine("Diff is currently: " + Math.abs(startStageY - worldPose.y));
//            if(Math.abs(startStageY - worldPose.y) > 5) {
//                DriveTrain.stopMovement();
//                nextStage();
//            }
//        }
//
//        if(progStage == progStages.turn.ordinal()) {
//            if(stageFinished) {
//                initProgVars();
//            }
//
//            DriveTrain.movementTurn = 0.3;
//            telemetry.addLine("Currently on stage: " + progStages.turn.name());
//            telemetry.addLine("Diff is currently: " + (Math.abs(Math.toDegrees(MathUtil.angleWrap(startStageHeading - worldPose.heading))) > 45));
//            telemetry.addLine("Diff is currently: " + Math.abs(Math.toDegrees(MathUtil.angleWrap(startStageHeading - worldPose.heading))));
//            if(Math.abs(Math.toDegrees(MathUtil.angleWrap(startStageHeading - worldPose.heading))) > 45) {
//                DriveTrain.stopMovement();
//                nextStage();
//            }
//        }

        if(progStage == progStages.goToPosition.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            PPController.goToPosition(12, 12, 0.5, this);

            if(Odometry.currPose.distanceBetween(new Pose(12,12,0)) < 1)
                DriveTrain.stopMovement();
        }

        if(progStage == progStages.stop.ordinal()) {
            requestOpModeStop();
        }
    }
}
