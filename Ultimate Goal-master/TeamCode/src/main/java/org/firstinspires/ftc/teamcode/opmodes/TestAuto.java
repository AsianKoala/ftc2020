package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.movement.CurvePoint;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.movement.Odometry.*;

@Disabled
@Autonomous(name="test")
public class TestAuto extends Auto {


    public enum programStates {
        purePursuit,
        returnToStart,
        turnAndEnd,
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
        setStage(programStates.purePursuit.ordinal());
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void MainStateMachine() {
        if(progState == programStates.purePursuit.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0, 0, 0, 0, 0, 0));
            allPoints.add(new CurvePoint(0, 15, 0.4, 0.4, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(5, 23, 0.4, 0.4, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(9, 27, 0.4, 0.4, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(24, 36, 0.4, 0.4, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(40, 40, 0.4, 0.4, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(60, 40, 0.4, 0.4, 20, 25, Math.toRadians(60), 0.6));
            boolean complete = PPController.followCurve(allPoints, Math.toRadians(90));

            if(complete) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progState == programStates.returnToStart.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            // curr: changed x -> 5 and slowDownTurnRadians -> 30/45
            // next test: have goToPosition slow down when it is <12 inches away from REAL LAST POINT

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(startStageX, startStageY, 0, 0, 0, 0, 0, 0));
            allPoints.add(new CurvePoint(40, 40, 0.4, 0.4, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(24, 36, 0.4, 0.4, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(9, 27, 0.4, 0.4, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(5, 23, 0.4, 0.4, 20, 25, Math.toRadians(60), 0.6));

            double scaleDownLastMove = (1.0 * Range.clip((-8-currentPosition.x)/(1.0 * (23+8)), 0.05, 1));
            allPoints.add(new CurvePoint(5, 15, 0.4 * scaleDownLastMove, 0.4, 20, 25, Math.toRadians(60), 0.9));
            allPoints.add(new CurvePoint(6, 10, 0.4 * scaleDownLastMove, 0.4, 20, 25, Math.toRadians(60), 0.9));
            allPoints.add(new CurvePoint(6, -8, 0.4 * scaleDownLastMove, 0.4, 20, 25, Math.toRadians(60), 0.9));
            boolean complete = PPController.followCurve(allPoints, Math.toRadians(90));

            if(complete) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progState == programStates.turnAndEnd.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            PPController.pointAngle(Math.toRadians(90), 0.5, Math.toRadians(30));

            if(Math.abs(MathUtil.angleWrap(Odometry.currentPosition.heading - Math.toRadians(90))) < Math.toRadians(2)) {
                DriveTrain.stopMovement();
                requestOpModeStop();
            }
        }


    }
}
