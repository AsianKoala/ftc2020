package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;

import static org.firstinspires.ftc.teamcode.hardware.BetterOdometry.worldPose;

@Autonomous
public class TestAuto1 extends MainAuto {

    Pose startPose = new Pose(0, 0, 0);

    public enum progStages {
        driveForward,
        turn,
        goToPosition,
        stop
    }

    @Override
    public void init() {
        super.init();
        betterOdometry.setWorldPose(startPose);
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
            telemetry.addLine("Currently on stage: " + progStages.driveForward.name());
            telemetry.addLine("Diff is currently: " + Math.abs(startStageY - worldPose.y));
            if(Math.abs(startStageY - worldPose.y) > 5) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progStage == progStages.turn.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            DriveTrain.movementTurn = 0.3;
            telemetry.addLine("Currently on stage: " + progStages.turn.name());
            telemetry.addLine("Diff is currently: " + (Math.abs(Math.toDegrees(MathUtil.angleWrap(startPose.heading - worldPose.heading))) > 45));
            telemetry.addLine("Diff is currently: " + Math.abs(Math.toDegrees(MathUtil.angleWrap(startPose.heading - worldPose.heading))));
            if(Math.abs(Math.toDegrees(MathUtil.angleWrap(startPose.heading - worldPose.heading))) > 45) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progStage == progStages.goToPosition.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            Pose targetPose = new Pose(12, 12, 0);
            PPController.goToPosition(targetPose.x, targetPose.y, 0.5);
            telemetry.addLine("Diff is currently: " + worldPose.distanceBetween(targetPose));

            if(worldPose.distanceBetween(targetPose) < 2)
                DriveTrain.stopMovement();
        }

        if(progStage == progStages.stop.ordinal()) {
            requestOpModeStop();
        }
    }
}
