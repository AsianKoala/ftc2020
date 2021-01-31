package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.hardware.BetterOdometry;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.hardware.OdometrySet;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubMotor;

import static org.firstinspires.ftc.teamcode.util.MathUtil.angleWrap;


public class Robot extends TunableOpMode {

    public DriveTrain driveTrain;

    // odom shit
    public Odometry odometry;
    public OdometrySet odometrySet;
    private BNO055IMU imu;
    private double headingOffset;
    private double lastHeading;

    BetterOdometry betterOdometry;


    @Override
    public void init() {
        Hardware.loadParentOpMode(this);

        ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;
        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");
        driveTrain = new DriveTrain(frontLeft, backLeft, frontRight, backRight);

        ExpansionHubMotor verticalOdometer = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        ExpansionHubMotor horizontalOdometer = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
        odometrySet = new OdometrySet(verticalOdometer, horizontalOdometer);
        odometry = new Odometry(odometrySet);
        initBNO055IMU(hardwareMap);

        betterOdometry = new BetterOdometry(odometrySet);
    }

    @Override
    public void init_loop() {
        telemetry.addLine(odometry.toString());
    }

    @Override
    public void loop() {
        driveTrain.update();

        lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        odometry.update(lastHeading);
        telemetry.addLine(odometry.toString());

        betterOdometry.update(lastHeading);
        telemetry.addLine(odometry.toString());
    }



    private void initBNO055IMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled  = false;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;
    }

}
