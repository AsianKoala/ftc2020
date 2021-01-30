package org.firstinspires.ftc.teamcode.main;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.openftc.revextensions2.ExpansionHubMotor;

public class Robot extends TunableOpMode {

    public DriveTrain driveTrain;
//    public Odometry odometer;

    @Override
    public void init() {
        Hardware.loadParentOpMode(this);

        ExpansionHubMotor frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        ExpansionHubMotor frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        ExpansionHubMotor backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        ExpansionHubMotor backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");
        driveTrain = new DriveTrain(frontLeft, backLeft, frontRight, backRight);

        ExpansionHubMotor horizontalOdometer = hardwareMap.get(ExpansionHubMotor.class, "horizontal");
        ExpansionHubMotor verticalOdometer = hardwareMap.get(ExpansionHubMotor.class, "vertical");
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
//        driveTrain.update();
//
//        telemetry.addData("frontLeft", driveTrain.frontLeft.getCurrentPosition());
//        telemetry.addData("frontRight", driveTrain.frontRight.getCurrentPosition());
//        telemetry.addData("backLeft", driveTrain.backLeft.getCurrentPosition());
//        telemetry.addData("backRight", driveTrain.backRight.getCurrentPosition());
    }
}
