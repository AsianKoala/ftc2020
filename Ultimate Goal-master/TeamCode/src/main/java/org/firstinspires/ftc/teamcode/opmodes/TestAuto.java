package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.movement.CurvePoint;
import static org.firstinspires.ftc.teamcode.movement.Odometry.*;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;

import java.util.ArrayList;


@Autonomous
public class TestAuto extends Auto {


    public enum progStages {
        driveForward,
        turn,
        purePursuit,
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
        setStage(progStages.purePursuit.ordinal());
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
            if(Math.abs(startStageY - currentPosition.y) > 5) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progStage == progStages.turn.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            DriveTrain.movementTurn = 0.3;
            if(Math.abs(Math.toDegrees(MathUtil.angleWrap(startStageHeading - currentPosition.heading))) > 45) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progStage == progStages.purePursuit.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }


//            PPController.goToPosition(36, 36, 0.75, Math.toRadians(90), 0.75, Math.toRadians(15), 0, false);
//            if(Odometry.currentPosition.distanceBetween(new Pose(-24,24,0)) < 1)
//                DriveTrain.stopMovement();

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.75, 0.75, 20, 20, Math.toRadians(30), 1.0));
            allPoints.add(new CurvePoint(9, 27, 0.75, 0.75, 20, 20, Math.toRadians(30), 1.0));
            allPoints.add(new CurvePoint(36, 36, 0.75, 0.75, 20, 20, Math.toRadians(30), 1.0));

            PPController.followCurve(allPoints, Math.toRadians(90));

            if(currentPosition.distance(new Point(36, 36)) < 1) {
                DriveTrain.stopMovement();
            }
        }

        if(progStage == progStages.stop.ordinal()) {
            requestOpModeStop();
        }
    }
}
