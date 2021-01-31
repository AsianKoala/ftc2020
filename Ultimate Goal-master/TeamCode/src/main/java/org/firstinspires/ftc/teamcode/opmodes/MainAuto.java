package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.BetterOdometry.worldPose;

import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.Pose;

public class MainAuto extends Robot {

    public int progStage;
    public boolean stageFinished;
    public int completedStages;

    public void setStage(int stage) {
        progStage = stage;
        stageFinished = true;
    }

    public void nextStage() {
        setStage(progStage + 1);
        completedStages++;
    }


    protected double startStageX;
    protected double startStageY;
    protected double startStageHeading;
    protected Pose startStagePose;
    protected void initProgVars() {
        startStageX = worldPose.x;
        startStageY = worldPose.y;
        startStageHeading = worldPose.heading;
        startStagePose = new Pose(startStageX, startStageY, startStageHeading);
        stageFinished = false;
        PPController.initCurve();
        PPController.initMove();
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
        progStage = 0;
        completedStages = 0;
        stageFinished = true;
    }

    @Override
    public void loop() {
        super.loop();
        MainStateMachine();
    }

    public void MainStateMachine() {

    }
}
