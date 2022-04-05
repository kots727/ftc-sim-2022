package org.firstinspires.ftc.teamcode.disabled_samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Swerve Complete", group = "Test")
public class TestSwerve extends LinearOpMode {

    SwerveDriveWPI bot = new SwerveDriveWPI();
    BNO055IMU imu;
    public void runOpMode(){

        bot.init(hardwareMap);
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        this.imu.initialize(parameters);
        waitForStart();
        while(opModeIsActive()) {
            double vx = -gamepad1.left_stick_x * SwerveDriveWPI.MAX_DRIVE_SPEED;
            double vy = -gamepad1.left_stick_y * SwerveDriveWPI.MAX_DRIVE_SPEED;
            double va = gamepad1.right_stick_x * SwerveDriveWPI.MAX_ANGULAR_SPEED;
            //float va = -gamepad1.right_stick_x;
            Orientation orientation = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", Math.toDegrees(orientation.firstAngle));
            telemetry.update();
            bot.drive(vy, vx, va,orientation.firstAngle);
        }
    }

}
