package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;

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
        odometry.setStartPosition(startPose);
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
            telemetry.addLine("Diff is currently: " + Math.abs(startStageY - Odometry.currY));
            if(Math.abs(startStageY - Odometry.currY) > 5) {
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
            telemetry.addLine("Diff is currently: " + (Math.abs(Math.toDegrees(MathUtil.angleWrap(startPose.heading - Odometry.currHeading))) > 45));
            telemetry.addLine("Diff is currently: " + Math.abs(Math.toDegrees(MathUtil.angleWrap(startPose.heading - Odometry.currHeading))));
            if(Math.abs(Math.toDegrees(MathUtil.angleWrap(startPose.heading - Odometry.currHeading))) > 45) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progStage == progStages.goToPosition.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

//            PPController.goToPosition(0, 24, 0, 0.5, 0.2, this);
            Pose targetPose = new Pose(24, 24, 0);
            PPController.goToPosition(targetPose.x, targetPose.y, 0.5);
            telemetry.addLine("Diff is currently: " + Odometry.currPose.distanceBetween(targetPose));
            if(Odometry.currPose.distanceBetween(targetPose) < 2) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progStage == progStages.stop.ordinal()) {
            requestOpModeStop();
        }
    }
}
