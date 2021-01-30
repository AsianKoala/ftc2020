package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.main.util.AxesSigns;
import org.firstinspires.ftc.teamcode.main.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.main.util.Pose;
import org.openftc.revextensions2.ExpansionHubMotor;

import static org.firstinspires.ftc.teamcode.main.util.MathUtil.angleWrap;

@TeleOp
public class Robot extends TunableOpMode {

    public DriveTrain driveTrain;

    // odom shit
    public Odometry odometry;
    public OdometrySet odometrySet;
    private BNO055IMU imu;
    private double headingOffset;
    private double lastHeading;

    public final Pose startPose = new Pose(25, 25, 0);

    @Override
    public void init() {
        Hardware.loadParentOpMode(this);

        ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;
        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");
        driveTrain = new DriveTrain(frontLeft, backLeft, frontRight, backRight);

        ExpansionHubMotor parallelOdometer = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        ExpansionHubMotor lateralOdometer = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
        odometrySet = new OdometrySet(parallelOdometer, lateralOdometer);
        odometry = new Odometry(startPose, odometrySet);
        initBNO055IMU(hardwareMap);
    }

    @Override
    public void loop() {
        controlMovement();
        driveTrain.update();

        lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        odometry.update(angleWrap(lastHeading));

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
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled  = false;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;
    }

}
