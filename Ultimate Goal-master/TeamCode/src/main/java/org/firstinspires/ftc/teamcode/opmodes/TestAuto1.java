package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;

@Autonomous
public class TestAuto1 extends MainAuto {

    public enum progStages {
        driveForward,
        turn,
        driveForwardAgain
    }

    @Override
    public void init() {
        super.init();
        odometry.setStartPosition(new Pose(25, 25, 0));
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
                requestOpModeStop();
            }

            DriveTrain.movementTurn = 0.5;
            telemetry.addLine("Currently on stage: " + progStages.turn.name());
            telemetry.addLine("Diff is currently: " + Math.abs(Math.toDegrees(MathUtil.angleWrap(startStageHeading - Odometry.currHeading))));
            telemetry.addLine("current heading in deg: " + Math.toDegrees(Odometry.currHeading));
            telemetry.addLine("start heading in deg: " + Math.toDegrees(startStageHeading));
            if(Math.abs(Math.toDegrees(MathUtil.angleWrap(startStageHeading - Odometry.currHeading))) > 45) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }
    }
}
