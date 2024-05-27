package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import static android.os.SystemClock.sleep;

/*
Software problems:
Remap Controls - each joystick controls it's respective wheel
Add brake mode to motors so gravity doesn't affect the arm
Robot too fast: Add pressure sensitivity.
Simplify Buttons
Make wrist move by hold the button instead of one press
Slow down pickup function
Set limit to how far the arm can go

Hardware problems:
Attach control hub and other electronics to base
Label Wires
Wire Management
add weight to front so it doesn't tip over when arm is out
Research Gear Ratio
zip ties are too long - cut off tails
Claw is too tight
Two game-pads - one for move and one for claw, wrist, and arm

Also scissor lift and lead screw should be use in case we ever plan on making the robot hang off a bar
 */

@TeleOp(name = "TeleOp", group = "Iterative Opmode")
public class teamTeleOpCode extends OpMode {
    
    public static Attachments iRobot = new Attachments();

    // Servos
    public static double wristPosition = Constants.wristUp;
    public static double clawPosition = Constants.clawOpen;
    private boolean initialized = false;

    int clicks = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        iRobot.initialize(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private double getMotorPower(double x) {
        double output = 0.9*(Math.pow(x,2));
        return Math.max(output, 0.2);
    }

    /*
     * Code to run after the driver hits PLAY but before they hit STOP
     * This is a continuous loop that responds to buttons being pressed on the game pad
     */
    @Override
    public void loop() {
        /* ------------------------------------ Drive ------------------------------------ */
        // Get Position constants
        double leftArmPosition = iRobot.getLeftArmPosition();
        double rightArmPosition = iRobot.getRightArmPosition();
        wristPosition  = iRobot.getWristPosition();
        clawPosition = iRobot.getClawPosition();
        double droneServoLocation = iRobot.droneServo.getPosition();
        // Motors
        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;

        float motorPower = 0.6f;
        int liftMax = 550;
        int liftMin = 0;
        int cycleLift1Pos = 0;

        if (initialized == false)
        {
            initialized = true;
            iRobot.setWristServo(Constants.wristDown + 0.1);
            iRobot.setClawServo(Constants.clawOpen);
            iRobot.setDroneServo(0);
        }

        /*
            Left Wheel Control
         */
        if(ly > 0.1)
        {
            iRobot.leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.leftDriveMotor.setPower(getMotorPower(ly));
        }
        else if(ly < -0.1) {
            iRobot.leftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.leftDriveMotor.setPower(getMotorPower(ly));
        }
        else
        {
            iRobot.leftDriveMotor.setPower(0);
        }

        /*
            Right Wheel Control
         */
        if(ry > 0.1)
        {
            iRobot.rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.rightDriveMotor.setPower(getMotorPower(ry));
        }
        else if(ry < -0.1) {
            iRobot.rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.rightDriveMotor.setPower(getMotorPower(ry));
        }
        else
        {
            iRobot.rightDriveMotor.setPower(0);
        }

        /*
            move wrist up and down when X & Y buttons are pressed
        */
        if (gamepad1.b)
        {
            if(wristPosition < Constants.wristUp) {
                wristPosition += 0.02;
                iRobot.wristServo.setPosition(wristPosition);

            }
            sleep(10);
        }
        if (gamepad1.a)
        {
            if(wristPosition > Constants.wristDown) {
                wristPosition -= 0.02;
                iRobot.wristServo.setPosition(wristPosition);

            }
            sleep(10);
       }


        /*
            open and close claw when A and B buttons are pressed
        */
        if (gamepad1.right_bumper) {
            clicks += 1;
            if (clicks == 2){
                if (clawPosition == Constants.clawOpen) {
                    clawPosition = Constants.clawClose;
                } else if (clawPosition == Constants.clawClose) {
                    clawPosition = Constants.clawOpen;
                }
                iRobot.clawServo.setPosition(clawPosition);
                sleep(200);
                clicks = 0;
            }
        }

            /*
        if (gamepad1.right_bumper) {
            if(clawPosition == Constants.clawOpen) {
                clawPosition = Constants.clawClose;

            }
            else if(clawPosition == Constants.clawClose) {
                clawPosition = Constants.clawOpen;

            }
            iRobot.clawServo.setPosition(clawPosition);
            sleep(200);
        } */

       /*
            Move arm up and down when right_stick is moved in y direction
        */

        if(gamepad1.y)
        {
            if(leftArmPosition >=liftMax) {
                iRobot.leftArmMotor.setPower(0);
                iRobot.rightArmMotor.setPower(0);
            }
            else {
                iRobot.leftArmMotor.setPower(motorPower);
                iRobot.rightArmMotor.setPower(motorPower);
                leftArmPosition = iRobot.leftArmMotor.getCurrentPosition();
            }
        }
        else if(gamepad1.x) {
            iRobot.leftArmMotor.setPower(-motorPower);
            iRobot.rightArmMotor.setPower(-motorPower);
            leftArmPosition = iRobot.leftArmMotor.getCurrentPosition();
        }
        else
        {
            iRobot.leftArmMotor.setPower(0);
            iRobot.rightArmMotor.setPower(0);
        }

        // Actually moves the claw and extension
        //iRobot.setClawServo(clawPosition);
        //iRobot.setWristServo(wristPosition);

        /* ------------------------------------Wrist Down ----------------------------------------*/

        if(gamepad1.left_bumper) {
            iRobot.pickUpPixel();
    	}

   /*     if(gamepad1.right_trigger > 0.1) {
            iRobot.release();
        } */

        if(gamepad1.dpad_down) {
            iRobot.rotateClockwise();
        }

        if(gamepad1.dpad_up) {
            iRobot.rotateCounterClockwise();
        }

        if (gamepad1.left_trigger > 0.3 && gamepad1.right_trigger > 0.3) {
            iRobot.setDroneServo(0.7);
        }
        else if (gamepad1.left_trigger > 0.3) {
            iRobot.wristDown();
        }

        /* ------------------------------------ Telemetry ------------------------------------ */
        // Telemetry is for debugging
        telemetry.addData("left arm position", leftArmPosition);
        telemetry.addData("right arm position", rightArmPosition);
        telemetry.addData("drone position", droneServoLocation);
        telemetry.addData("claw position", clawPosition);
        telemetry.addData("wrist position", wristPosition);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}






