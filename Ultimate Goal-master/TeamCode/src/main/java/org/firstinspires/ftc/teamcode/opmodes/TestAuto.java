package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.movement.CurvePoint;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;


@Autonomous
public class TestAuto extends Auto {


    public enum progStages {
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
        setStage(progStages.purePursuit.ordinal());
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void MainStateMachine() {
        super.MainStateMachine();

        if(progStage == progStages.purePursuit.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }


            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0, 0, 0, 0, 0, 0));
            allPoints.add(new CurvePoint(0, 15, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(5, 23, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(9, 27, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(24, 36, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(60, 40, 0.5, 0.5, 20, 15, Math.toRadians(30), 0.6));
            boolean complete = PPController.followCurve(allPoints, Math.toRadians(90));

            if(complete) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progStage == progStages.returnToStart.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(startStageX, startStageY, 0, 0, 0, 0, 0, 0));
            allPoints.add(new CurvePoint(24, 36, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(9, 27, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(5, 23, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(0, 15, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(5, -7, 0.4, 0.6, 20, 15, Math.toRadians(30), 0.6));
            boolean complete = PPController.followCurve(allPoints, Math.toRadians(90));

            if(complete) {
                DriveTrain.stopMovement();
                nextStage();
            }
        }

        if(progStage == progStages.turnAndEnd.ordinal()) {
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
