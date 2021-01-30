package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.openftc.revextensions2.ExpansionHubMotor;

@TeleOp
public class TestTeleOp extends OpMode {

    DriveTrain driveTrain;
    ExpansionHubMotor leftIntake, rightIntake; // also used for odom shit

    // odom shit
    BNO055IMU imu;
    double headingOffset;
    double lastHeading;
    Odometry odometry;


    @Override
    public void init() {
        Hardware.loadParentOpMode(this);
        ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;
        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");
        driveTrain = new DriveTrain(frontLeft, backLeft, frontRight, backRight);

        leftIntake = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
        odometry = new Odometry();
        initBNO055IMU(hardwareMap);
    }

    @Override
    public void loop() {
        controlMovement();
        driveTrain.update();

        lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        odometry.update(leftIntake.getCurrentPosition(), rightIntake.getCurrentPosition(), lastHeading);

        telemetry.addLine();
        telemetry.addLine(odometry.toString());
        telemetry.update();
    }



    public void controlMovement() {
        double masterScale = 0.5 + ((gamepad1.right_bumper ? 1 : 0) * (1.0-0.5));
        DriveTrain.movementY = -gamepad1.left_stick_y * masterScale;// * getDouble("y_move_scale");
        DriveTrain.movementX = gamepad1.left_stick_x * masterScale;// * getDouble("x_move_scale");
        DriveTrain.movementTurn = -gamepad1.right_stick_x * masterScale;// * getDouble ("rot_move_scale");

    }

    private void initBNO055IMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu"); // @TODO find out correct axes
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled  = false;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;
    }

}
