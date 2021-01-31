package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;

@Autonomous
public class TestAuto1 extends MainAuto {

    Pose startPose = new Pose(0, 0, 0);

    public enum progStages {
        driveForward,
        turn,
        stop
    }

    @Override
    public void init() {
        super.init();
        odometry.setStartPosition(startPose);
    }

    @Override
    public void start() {
        super.start();
        setStage(progStages.turn.ordinal());
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

        if(progStage == progStages.stop.ordinal()) {
            requestOpModeStop();
        }
    }
}
