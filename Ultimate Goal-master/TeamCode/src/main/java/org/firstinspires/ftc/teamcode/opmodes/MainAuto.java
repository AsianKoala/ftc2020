package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.hardware.Odometry;
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
