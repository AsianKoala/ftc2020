package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;

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

        setStage(progStages.driveForward.ordinal());
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    void MainStateMachine() {

        if(getCurrStage() == progStages.driveForward.ordinal()) {
            if(isStageFinished()) {
                initProgVars();
            }

            DriveTrain.movementY = 0.5;
            if(Math.abs(startStageY - Odometry.currY) > 5) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(getCurrStage() == progStages.turn.ordinal()) {
            if(isStageFinished()) {
                requestOpModeStop();
            }

            DriveTrain.movementTurn = 0.5;
            if(Math.abs(Math.toDegrees(MathUtil.angleWrap(startStageHeading - Odometry.currHeading))) > 45) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }
    }
}
