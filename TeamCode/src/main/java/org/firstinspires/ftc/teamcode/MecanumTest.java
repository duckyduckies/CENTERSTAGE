package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static java.lang.Math.*;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import static android.os.SystemClock.sleep;
import java.util.List;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="mecanumtest")
public class MecanumTest extends LinearOpMode {


    int clicks = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot();
        robot.initialize(hardwareMap,telemetry);



        boolean checkForRed = false;
        boolean checkForBlue = false;
        // mode 0 : scanning
        // mode 1 : approaching

        double desiredDistance = 7;
        int alliance = 1;
        int liftMax = 550;
        int wristMax = 550;
        double leftArmPosition = robot.getLeftArmPosition();
        double wristPosition = robot.getWristPosition();

        double clawPosition = Constants.clawOpen;

        // 0 : blue
        // 1 : red


        double distance = Double.MAX_VALUE;

        waitForStart();

        while(opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            //make sure ^^ is negated
            double turn = gamepad1.right_stick_x;

            int red = robot.colorSensor.red();   // Red channel value
            int green = robot.colorSensor.green(); // Green channel value
            int blue = robot.colorSensor.blue();  // Blue channel value
            int alpha = robot.colorSensor.alpha(); // Total luminosity
            int argb = robot.colorSensor.argb();  // Combined color value

            robot.move(x,y,turn,1);





            if(gamepad1.dpad_up) {
                robot.move(0,1,0,0.5);
            }
            if(gamepad1.dpad_down) {
                robot.move(0,-1,0,0.5);
            }
            if(gamepad1.dpad_left) {
                robot.move(-1,0,0,0.5);
            }
            if(gamepad1.dpad_right) {
                robot.move(1,0,0,0.5);
            }



            /*
            move arm up and down when x & y buttons are pressed
        */

            if(gamepad1.y)
            {
                if(leftArmPosition >=liftMax) {
                    robot.motor1ex.setPower(0);
                    robot.motor2ex.setPower(0);
                }
                else {
                    robot.motor1ex.setPower(0.6);
                    robot.motor2ex.setPower(0.6);
                    leftArmPosition = robot.motor1ex.getCurrentPosition();
                }
            }
            else if(gamepad1.x) {
                robot.motor1ex.setPower(-0.6);
                robot.motor2ex.setPower(-0.6);
                leftArmPosition = robot.motor1ex.getCurrentPosition();
            }
            else
            {
                robot.motor1ex.setPower(0);
                robot.motor2ex.setPower(0);
            }

         /*
            move wrist up and down when a & b buttons are pressed
        */
            if (gamepad1.b)
            {
                if(wristPosition < Constants.wristUp) {
                    wristPosition += 0.02;
                    robot.servo1.setPosition(wristPosition);

                }
                sleep(10);
            }
            if (gamepad1.a)
            {
                if(wristPosition > Constants.wristDown) {
                    wristPosition -= 0.02;
                    robot.servo1.setPosition(wristPosition);

                }
                sleep(10);
            }


        /*
            open and close one claw
        */
            if (gamepad1.left_bumper) {
                clicks += 1;
                if (clicks == 2){
                    if (clawPosition == Constants.clawOpen) {
                        clawPosition = Constants.clawClose;
                    } else if (clawPosition == Constants.clawClose) {
                        clawPosition = Constants.clawOpen;
                    }
                    robot.servo3.setPosition(clawPosition);
                    sleep(200);
                    clicks = 0;
                }
            }
        /*
            open and close the other claw
        */
            if (gamepad1.right_bumper) {
                clicks += 1;
                if (clicks == 2){
                    if (clawPosition == Constants.clawOpen) {
                        clawPosition = Constants.clawClose;
                    } else if (clawPosition == Constants.clawClose) {
                        clawPosition = Constants.clawOpen;
                    }
                    robot.servo2.setPosition(clawPosition);
                    sleep(200);
                    clicks = 0;
                }
            }

            // move the wheel out
            if (gamepad1.left_trigger > 0.3){
                robot.motor3ex.setPower(0.5);
            }

            // move the wheel in
            if (gamepad1.right_trigger > 0.3){
                robot.motor3ex.setPower(-0.5);
            }

            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                if (clawPosition == Constants.clawOpen) {
                    clawPosition = Constants.clawClose;
                } else if (clawPosition == Constants.clawClose) {
                    clawPosition = Constants.clawOpen;
                }
                robot.servo3.setPosition(clawPosition);
                robot.servo2.setPosition(clawPosition);
                sleep(200);
                clicks = 0;

            }


            if (gamepad1.left_trigger > 0.3 && gamepad1.right_trigger > 0.3) {
                robot.servo4.setPosition(0.5);
            }

            //if (gamepad1.start) {
            //    robot.servo4
            //}

           /* if(gamepad1.right_bumper) {
                aprilTagRunning = true;
            }
            if(gamepad1.x) {
                checkForRed = true;
            }
            if(gamepad1.y) {
                checkForBlue = true;
            }

            if(aprilTagRunning) {
                if (aprilTagDetected && aprilTagMode == 0) {
                    aprilTagMode = 1;
                }
                else if (aprilTagDetected && aprilTagMode == 1) {
                    double difference = distance - desiredDistance;
                    // estimating that it takes 170 ms for robot to move 1 inch forward (power 0.15)
                    if (difference > 0.1) {
                        robot.move(0, 1, 0, 0.15);
/////////////////////////going up

                        sleep((long) (170 * difference));
                    } else if (difference < -0.1) {
                        robot.move(0, -1, 0, 0.15);
/////////////////////////going down
                        sleep((long) (170 * abs(difference)));
                    }
                    aprilTagRunning = false;
                    aprilTagMode = 0;

                    if (alliance == 0) {
                        robot.move(1, 0, 0, 0.2);
                        sleep(750);
                    } else if (alliance == 1) {
                        robot.move(1, 0, 0, 0.2);
                        sleep(300);
                    }
                }
                else if (aprilTagDetected == false && aprilTagMode == 0) {
                    if (alliance == 0)
                    {
                        robot.move(1,0,0,0.3);
                    }
                    else if (alliance == 1)
                    {
                        robot.move(-1,0,0,0.3);
                    }
                }
            }
            if(checkForRed) {
                if(red < robot.default_red + 500) {
                    robot.move(0,1,0,0.3);
                }
                else  {
                    checkForRed = false;
                }
            }

            if(checkForBlue) {
                if (blue < robot.default_blue + 500) {
                    robot.move(0, 1, 0, 0.3);
                } else {
                    checkForBlue = false;
                }
            }


            telemetry.addData("Red: ", red);
            telemetry.addData("Blue: ", blue);
            telemetry.addData("Checking for Red: ", checkForRed);
            telemetry.addData("Checking for Blue: ", checkForBlue);
            telemetry.addData("distance",distance);
            telemetry.update(); */
        }

    }



}




