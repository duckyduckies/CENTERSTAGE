package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "Iterative Opmode")
public class teamTeleOpCode extends OpMode {
    
    private Attachments iRobot = new Attachments();


    private double currentLift1Position = 0;

    private double currentLift2Position = 0;


    // Servos

    private double armPosition = Constants.armIn;
    private double clawPosition = Constants.clawOpen;


    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        iRobot.initialize(hardwareMap);
        //currentClawPosition = iRobot.clawServo.getPosition();
        //currentArmPosition = iRobot.armServo.getPosition();
        // Tell the driver that initialization is complete.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }



    /*
     * Code to run after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /* ------------------------------------ Drive ------------------------------------ */
        // Position constants
        /*
        currentLift1Position = iRobot.getLiftMotor1Position();
        currentLift2Position = iRobot.getLiftMotor2Position();
        currentArmPosition = iRobot.getArmPosition();
        currentClawPosition = iRobot.getClawPosition();
         */

        // Motors
        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;

        float motorPower = 1.0f;
        int liftMax = -1175;
        int liftMin = 0;
        int cycleLift1Pos = 0;

        /*
            Move forward or backwards
         */
        if(ly > 0.1)
        {
            iRobot.leftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.leftDriveMotor.setPower(motorPower);
            iRobot.rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.rightDriveMotor.setPower(motorPower);
        }
        else if(ly < -0.1) {
            iRobot.leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.leftDriveMotor.setPower(motorPower);
            iRobot.rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.rightDriveMotor.setPower(motorPower);
        }
        else
        {
            iRobot.leftDriveMotor.setPower(0);
            iRobot.rightDriveMotor.setPower(0);
        }

        /*
            Turn left or right
         */
        if(lx > 0.1)
        {
            iRobot.leftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.leftDriveMotor.setPower(motorPower);
            iRobot.rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.rightDriveMotor.setPower(motorPower);
        }
        else if(lx < -0.1) {
            iRobot.leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.leftDriveMotor.setPower(motorPower);
            iRobot.rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.rightDriveMotor.setPower(motorPower);
        }
        else
        {
            iRobot.leftDriveMotor.setPower(0);
            iRobot.rightDriveMotor.setPower(0);
        }


        /*
            arm control
        */
        if (gamepad1.y)
        {
            armPosition = Constants.armIn;
            iRobot.armServo.setPosition(armPosition);
        }
        if (gamepad1.x)
        {
            armPosition = Constants.armOut;
            iRobot.armServo.setPosition(armPosition);
        }

        /*
            claw control
        */
        if (gamepad1.a) {
            clawPosition = Constants.clawOpen;
            iRobot.clawServo.setPosition(clawPosition);
        } else if (gamepad1.b) {
            clawPosition = Constants.clawClose;
            iRobot.clawServo.setPosition(clawPosition);
        }

        /*
            lift control
        */
        if(ry > 0.1)
        {
            iRobot.liftMotor1.setDirection(DcMotor.Direction.FORWARD);
            iRobot.liftMotor1.setPower(motorPower);
            iRobot.liftMotor2.setDirection(DcMotor.Direction.REVERSE);
            iRobot.liftMotor2.setPower(motorPower);
        }
        else if(ry < -0.1) {
            iRobot.liftMotor1.setDirection(DcMotor.Direction.REVERSE);
            iRobot.liftMotor1.setPower(motorPower);
            iRobot.liftMotor2.setDirection(DcMotor.Direction.FORWARD);
            iRobot.liftMotor2.setPower(motorPower);
        }
        else
        {
            iRobot.liftMotor1.setPower(0);
            iRobot.liftMotor2.setPower(0);
        }


        // Actually moves the claw and extension
        //iRobot.setClawServo(clawPosition);
        //iRobot.setArmServo(armPosition);






        /* ------------------------------------ Telemetry ------------------------------------ */

        // Telemetry is for debugging
        telemetry.addData("arm position", armPosition);
        telemetry.addData("claw position", clawPosition);
        telemetry.update();

    }

    @Override
    public void stop() {
    }





}
