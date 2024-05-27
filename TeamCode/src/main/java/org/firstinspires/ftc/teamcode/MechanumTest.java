package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MechanumTest", group = "Iterative Opmode")
public class MechanumTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motor1");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motor2");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motor3");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motor4");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX + rx) / denominator) * -1;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = ((rotY - rotX - rx) / denominator) * -1;
            double backRightPower = ((rotY + rotX - rx) / denominator) * -1;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if(gamepad1.dpad_up) {
                frontLeftMotor.setPower(-1);
                backLeftMotor.setPower(1);
                frontRightMotor.setPower(-1);
                backRightMotor.setPower(-1);
            }
            if(gamepad1.dpad_down) {
                frontLeftMotor.setPower(1);
                backLeftMotor.setPower(-1);
                frontRightMotor.setPower(1);
                backRightMotor.setPower(1);
            }
            if(gamepad1.dpad_left) {
                frontLeftMotor.setPower(1);
                backLeftMotor.setPower(1);
                frontRightMotor.setPower(-1);
                backRightMotor.setPower(1);
            }
            if(gamepad1.dpad_right) {
                frontLeftMotor.setPower(-1);
                backLeftMotor.setPower(-1);
                frontRightMotor.setPower(1);
                backRightMotor.setPower(-1);
            }
            telemetry.addData("Left Joystick x", x);
            telemetry.addData("Left Joystick y", y);
            telemetry.addData("Right Joystick x", rx);
            telemetry.addData("Back Right Wheel Power", backRightPower);
            telemetry.addData("Back Left Wheel Power", backLeftPower);
            telemetry.addData("Front Right Wheel Power", frontRightPower);
            telemetry.addData("Front Left Wheel Power", frontLeftPower);

            telemetry.update();
        }
    }
}