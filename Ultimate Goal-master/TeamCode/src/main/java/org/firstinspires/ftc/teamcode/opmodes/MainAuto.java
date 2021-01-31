package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.Pose;

public abstract class MainAuto extends Robot {

    private int progStage;
    private boolean stageFinished;

    public void setStage(int stage) {
        progStage = stage;
        stageFinished = true;
    }

    public void nextStage() {
        setStage(progStage + 1);
    }

    public int getCurrStage() {
        return progStage;
    }

    public boolean isStageFinished() {
        return stageFinished;
    }

    protected double startStageX;
    protected double startStageY;
    protected double startStageHeading;
    protected void initProgVars() {
        startStageX = Odometry.currX;
        startStageY = Odometry.currY;
        startStageHeading = Odometry.currHeading;
        stageFinished = false;
        PPController.initCurve();
        PPController.initMove();
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        progStage = 0;
        stageFinished = false;
    }

    @Override
    public void loop() {
        super.loop();
        MainStateMachine();
    }

    abstract void MainStateMachine();
}
