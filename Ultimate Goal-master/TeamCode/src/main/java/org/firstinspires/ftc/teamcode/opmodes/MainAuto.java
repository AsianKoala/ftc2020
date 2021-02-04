package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.movement.CurvePoint;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.opmodes.Auto.RingDetectorPipeline.*;

import java.util.ArrayList;

public class MainAuto extends Auto {
    public enum programStates {
        initialShots,
        collectRings,
        secondShots,
        placeMarker,
        goToSecondMarker,
        pickUpMarker,
        placeSecondMarker,
        park
    }

    RingAmount ringAmount;

    @Override
    public void init() {
        super.init();
        odometry.setStart(new Pose(24,0, Math.toRadians(0)));
        setStage(programStates.initialShots.ordinal());
    }

    @Override
    public void init_loop() {
        super.init_loop();
        ringAmount = pipeline.getRingAmount();
    }

    @Override
    public void start() {
        super.start();
        setStage(programStates.initialShots.ordinal());
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void MainStateMachine() {
        if(progState == programStates.initialShots.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(startStageX, startStageY, 0, 0, 0, 0, 0, 0));
            allPoints.add(new CurvePoint(0, 15, 0.3, 0.3, 15, 20, Math.toRadians(60), 0.7));
        }
    }
}
