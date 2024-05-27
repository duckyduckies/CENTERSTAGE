package org.firstinspires.ftc.teamcode;

import static java.lang.Math.*;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.lynx.LynxModule;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="MecanumTest")
public class MecanumTest extends LinearOpMode {

    MecanumRobot robot = new MecanumRobot(this);
    boolean debugMode = true;

    private class DriveThread extends Thread
    {
        public DriveThread()
        {
            this.setName("DriveThread");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
             try
            {
                double x,y,turn;
                while (opModeIsActive() && !isInterrupted())
                {
                    /*
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.

                    leftY = gamepad1.left_stick_y * -1;
                    rightY = gamepad1.right_stick_y * -1;

                    leftMotor.setPower(Range.clip(leftY, -1.0, 1.0));
                    rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));
                    */
                    // Mecanum Drivetrain By Kush & Derek 11/18-11/22

                    x = gamepad1.left_stick_x;
                    y = -gamepad1.left_stick_y;
                    //make sure ^^ is negated
                    turn = gamepad1.right_stick_x;

                    robot.move(x, y, turn, 1);

                    if (gamepad1.dpad_up) {
                        robot.move(0, 1, 0, 0.5);
                    }
                    if (gamepad1.dpad_down) {
                        robot.move(0, -1, 0, 0.5);
                    }
                    if (gamepad1.dpad_left) {
                        robot.move(-1, 0, 0, 0.5);
                    }
                    if (gamepad1.dpad_right) {
                        robot.move(1, 0, 0, 0.5);
                    }

                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            //catch (InterruptedException e) {telemetry.addData("%s interrupted", this.getName());}
            // an error occurred in the run loop.
            catch (Exception e) {e.printStackTrace();}
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize();
        telemetry.addData("Status", "Initialized");

        Thread  driveThread = new DriveThread();

        telemetry.addData("Mode", "waiting");

        // wait for start button.

        waitForStart();

        telemetry.addData("Status", "Started");

        // start the driving thread.

        driveThread.start();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        // manual mode
        double leftPower = 0;
        double rightPower = 0;
        double slidePower = 0;
        double leftPosition = 0;
        double rightPosition = 1;
        double wristPosition = 0;
        double launcherPosition = 0;
        boolean armDown = false;
        int autoArmDownState = 0;
        int autoArmUpState = 0;
        int leftArmPosition;
        int slidePosition;

        double dl,dr;

        /* Auto mode
        * April Tag
         */

        // int myAprilTaIdCode = -1; // not used
        // int targetAprilTag = 2; // not used

        // mode 0 : scanning
        // mode 1 : approaching
        int aprilTagMode = 0;
        boolean aprilTagRunning = Constants.aprilTagRunning;
        boolean aprilTagDetected = false;

        boolean checkForRed = false;
        boolean checkForBlue = false;
        // mode 0 : scanning
        // mode 1 : approaching 

        double desiredDistance = 7;
        int alliance = 1;

        // 0 : blue
        // 1 : red
        int red = 0;
        int blue = 0;
        int redL = 0;
        int blueL = 0;

        double distance = Double.MAX_VALUE;

        while(opModeIsActive()) {

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

/*
            // Mecanum Drivetrain By Kush & Derek 11/18-11/22

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            //make sure ^^ is negated
            double turn = gamepad1.right_stick_x;

            robot.move(x, y, turn, 1);

            if (gamepad1.dpad_up) {
                robot.move(0, 1, 0, 0.5);
            }
            if (gamepad1.dpad_down) {
                robot.move(0, -1, 0, 0.5);
            }
            if (gamepad1.dpad_left) {
                robot.move(-1, 0, 0, 0.5);
            }
            if (gamepad1.dpad_right) {
                robot.move(1, 0, 0, 0.5);
            }
*/

            // Touch sensor
            // Cannot use port 0 on driver station
            if (robot.touchSensor.isPressed()) {
                if (debugMode)
                    telemetry.addData("Touch Sensor", "Is Pressed");
                armDown = true;
                robot.stopAndResetArmSlide();
                robot.runWithoutEncoderArm();
                robot.runWithoutEncoderSlide();
            } else {
                if (debugMode)
                    telemetry.addData("Touch Sensor", "Is Not Pressed");
                armDown = false;
            }

            if (debugMode) {
                // Color Sensor
                blue = robot.getColorSensorBlue();
                blueL = robot.getLeftColorSensorBlue();
                red = robot.getColorSensorRed();
                redL = robot.getLeftColorSensorRed();

                telemetry.addData("Right Blue: ", blue);
                telemetry.addData("Right Red: ", red);
                telemetry.addData("Left Blue: ", blueL);
                telemetry.addData("Left Red: ", redL);
                telemetry.addData("Right Blue Default: ", robot.getDefaultBlue());
                telemetry.addData("Right Red Default: ", robot.getDefaultRed());
                telemetry.addData("Left Blue Default: ", robot.getLeftDefaultBlue());
                telemetry.addData("Left Red Default: ", robot.getLeftDefaultRed());
                //telemetry.addData("Right Blue Threshold: ", MecanumRobot.blue_threshold);
                //telemetry.addData("Right Red Threshold: ", MecanumRobot.red_threshold);
                //telemetry.addData("Left Blue Threshold: ", MecanumRobot.blue_threshold_left);
                //telemetry.addData("Left Red Threshold: ", MecanumRobot.red_threshold_left);

                // Distance Sensor
                telemetry.addData("Left Distance Sensor", String.format("%.01f cm", robot.distanceSensorL.getDistance(DistanceUnit.CM)));
                telemetry.addData("Right Distance Sensor", String.format("%.01f cm", robot.distanceSensorR.getDistance(DistanceUnit.CM)));
                telemetry.addData("Left Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawL.getDistance(DistanceUnit.CM)));
                telemetry.addData("Right Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawR.getDistance(DistanceUnit.CM)));
            }
            dl = robot.distanceSensorClawL.getDistance(DistanceUnit.CM);
            dr = robot.distanceSensorClawR.getDistance(DistanceUnit.CM); // middle
            if (dr > 30) {
                if (dl <= 26) {
                    if (wristPosition != MecanumRobot.wristDown) {
                        telemetry.addData("Pixel detected, change pattern", 0);
                        if (!robot.getPattern().equals(MecanumRobot.greenPattern)) {
                            robot.displayPattern(MecanumRobot.greenPattern);
                            telemetry.addData("Change pattern to", "green");
                        }
                    }
                } else {
                    telemetry.addData("Nothing detected", 0);
                    if (!robot.getPattern().equals(MecanumRobot.defaultPattern)) {
                        robot.displayPattern(MecanumRobot.defaultPattern);
                        telemetry.addData("Change pattern to", "default");
                    }// what?
                }
            }
            /*
            else if (dr < 15) { // incorrect
                if (wristPosition != MecanumRobot.wristDown) {
                    telemetry.addData("Too close, change pattern", 0);
                    if (!robot.getPattern().equals(MecanumRobot.redPattern)) {
                        robot.displayPattern(MecanumRobot.redPattern);
                        telemetry.addData("Change pattern to", "red");
                    }
                }
            }
            else if (dr >= 20 && dr <= 30)  { // incorrect
                if (wristPosition != MecanumRobot.wristDown) {
                    telemetry.addData("Close, change pattern", 0);
                    if (!robot.getPattern().equals(MecanumRobot.yellowPattern)) {
                        robot.displayPattern(MecanumRobot.yellowPattern);
                        telemetry.addData("Change pattern to", "yellow");
                    }
                }
            }
            */
            else {
                telemetry.addData("Nothing detected", 0);
                if (!robot.getPattern().equals(MecanumRobot.defaultPattern)) {
                    robot.displayPattern(MecanumRobot.defaultPattern);
                    telemetry.addData("Change pattern to", "default");
                }
            }
            /*
            * If NOT running the auto mode, then allows to control the arm, slide, wrist & claw
             */
            //if (autoArmDownState==0 && autoArmUpState==0) {

                /*
                // Arm movement by gamepad 1
                // up
                leftArmPosition = robot.getMotorPositionLeftArm();
                telemetry.addData("Left Arm Position", leftArmPosition);
                if (gamepad1.y) {
                    if (leftArmPosition < robot.armMax) {
                        leftPower = 0.4;
                        rightPower = 0.4;
                        robot.setMotorPowerArm(leftPower);
                        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                    }
                } //down
                else if (gamepad1.x) {
                    if (!armDown) {
                        leftPower = -0.2;
                        rightPower = -0.2;
                        robot.setMotorPowerArm(leftPower);
                        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                    }
                }
                */

                // Arm movement gamepad 2
                leftArmPosition = robot.getMotorPositionLeftArm();
                if (gamepad2.right_stick_y > 0) { // Move arm down
                    if (debugMode)
                        telemetry.addData("Right Joystick:", gamepad2.right_stick_y);
                    if (leftArmPosition <= MecanumRobot.armMin) {
                        robot.setMotorPowerArm(0); // brake
                        if (debugMode)
                            telemetry.addData("Lower arm. Brake arm motor", 0);
                    } else {
                        leftPower = -gamepad2.right_stick_y / 3;
                        robot.setMotorPowerArm(leftPower);
                        if (debugMode)
                            telemetry.addData("Lower arm. Set Arm Power to ", leftPower);
                    }
                } else if (gamepad2.right_stick_y < 0) { // Move arm up
                    if (debugMode)
                        telemetry.addData("Right Joystick:", gamepad2.right_stick_y);
                    if (leftArmPosition >= MecanumRobot.armMax) {
                        robot.setMotorPowerArm(0); // brake
                        if (debugMode)
                            telemetry.addData("Raise arm. Brake arm motor", 0);
                    } else {
                        leftPower = -gamepad2.right_stick_y / 3;
                        robot.setMotorPowerArm(leftPower);
                        if (debugMode)
                            telemetry.addData("Raise arm. Set Arm Power to ", leftPower);
                    }

                } else { // when nothing is pressed, brake the arm motors
                    leftPower = 0;
                    rightPower = 0;
                    robot.setMotorPowerArm(leftPower);
                }
                if (debugMode)
                    telemetry.addData("Arm New Position", leftArmPosition);

                /*
                 * Slide movement
                 * It's against intuition that the joystick gives a negative value when it is pushed up
                 */
                slidePosition = robot.getMotorPositionSlide();
                if (gamepad2.left_stick_y > 0) { // joystick down & retract
                    if (slidePosition <= MecanumRobot.slideMin) {
                        robot.setMotorPowerSlide(0); // brake
                        if (debugMode)
                            telemetry.addData("Extend arm. Brake slide motor", 0);
                    } else {
                        slidePower = -gamepad2.left_stick_y / 2;
                        robot.setMotorPowerSlide(slidePower);
                        if (debugMode)
                            telemetry.addData("Retract arm. Set Slide Power to ", slidePower);
                    }
                } else if (gamepad2.left_stick_y < 0) { // joystick up & extend
                    if (slidePosition >= MecanumRobot.slideMax) {
                        robot.setMotorPowerSlide(0); // brake
                        if (debugMode)
                            telemetry.addData("Extend arm. Brake slide motor", 0);
                    } else {
                        slidePower = -gamepad2.left_stick_y / 2;
                        robot.setMotorPowerSlide(slidePower);
                        if (debugMode)
                            telemetry.addData("Extend arm. Set Slide Power to ", slidePower);
                    }

                } else { // brake
                    robot.setMotorPowerSlide(0);
                }
                if (debugMode)
                    telemetry.addData("Slide New Position", slidePosition);


                // Hand movement

                // Make it harder to lose pixels
                // Only pressing unlock button plus a trigger will open the claws
                // Two pixels need to fall next to each other on the backdrop
                // So when releasing the pixels, the claws only half open
                leftPosition = robot.getServoPositionLeftHand();
                rightPosition = robot.getServoPositionRightHand();
                if (gamepad2.left_trigger > 0.9) {
                    if ((leftPosition == 0) && (gamepad2.a)) { //close
                        leftPosition = 0.4; //open
                        robot.setServoPositionLeftHand(leftPosition);
                        sleep(100);
                    } else {
                        leftPosition = 0; //close
                        robot.setServoPositionLeftHand(leftPosition);
                        sleep(100);
                    }
                    if (debugMode)
                        telemetry.addData("Claw Servos", "left (%.2f), right (%.2f)", leftPosition, rightPosition);
                }
                if (gamepad2.right_trigger > 0.9) {
                    if ((rightPosition == 1) && (gamepad2.a)) { //close
                        rightPosition = 0.6; //open
                        robot.setServoPositionRightHand(rightPosition);
                        sleep(100);
                    } else {
                        rightPosition = 1; //close
                        robot.setServoPositionRightHand(rightPosition);
                        sleep(100);
                    }
                }

                if (gamepad1.left_trigger > 0.9) {
                    if (leftPosition == 0) { //close
                        leftPosition = 0.4; //open
                        robot.setServoPositionLeftHand(leftPosition);
                        sleep(100);
                    } else {
                        leftPosition = 0; //close
                        robot.setServoPositionLeftHand(leftPosition);
                        sleep(100);
                    }
                    if (debugMode)
                        telemetry.addData("Claw Servos", "left (%.2f), right (%.2f)", leftPosition, rightPosition);
                }
                if (gamepad1.right_trigger > 0.9) {
                    if (rightPosition == 1) {
                        rightPosition = 0.6; //open
                        robot.setServoPositionRightHand(rightPosition);
                        sleep(100);
                    } else {
                        rightPosition = 1; //close
                        robot.setServoPositionRightHand(rightPosition);
                        sleep(100);
                    }
                }
                if (debugMode == true)
                    telemetry.addData("Claw Servos", "left (%.2f), right (%.2f)", leftPosition, rightPosition);

                // Wrist movement
                wristPosition = robot.getServoPositionWrist();
                if (gamepad2.dpad_down) { //wrist down
                    if (wristPosition < 1) {
                        wristPosition += 0.05;
                        robot.setServoPositionWrist(wristPosition);
                    }
                }
                else if (gamepad2.dpad_up) { //wrist up
                    if (wristPosition > 0) {
                        wristPosition -= 0.05;
                        robot.setServoPositionWrist(wristPosition);
                    }
                }
                if (debugMode)
                    telemetry.addData("Wrist Servo", "%.2f", wristPosition);

                // Auto arm down to go back to driving
                if (gamepad2.x) {
                    if (!armDown) {
                        robot.AutoArmDown();
                        robot.runWithoutEncoderSlide();
                        robot.runWithoutEncoderArm();
                        //autoArmDownState=1;
                    }
                }
                // Auto arm up to release pixels
                if (gamepad2.y) {
                    robot.AutoArmUp();
                    //robot.AutoArmUpBack();
                    robot.runWithoutEncoderSlide();
                    robot.runWithoutEncoderArm();
                    //autoArmUpState=1;
                }
                if (gamepad1.x)
                    robot.AutoWristDown();
                if (gamepad1.y)
                    robot.AutoWristUp();
            //}

            //Launcher
            if ((gamepad1.back) || (gamepad2.back)) {
                launcherPosition = 1;
                robot.setServoPositionLauncher(launcherPosition);
            }

            if (gamepad1.a) {
                robot.AutoSlidePickup();
                robot.runWithoutEncoderSlide();
                robot.runWithoutEncoderArm();
            }

            /*
            * AUTOMATIC functions - finite state machine
             */
            /*
            // Auto arm down to go back to driving
            switch (autoArmDownState) {
                case 0:
                    // do nothing
                    break;
                case 1:
                    // Wrist up
                    robot.setServoPositionWrist(MecanumRobot.defaultWristPosition);
                    autoArmDownState = 2;
                    break;
                case 2:
                    // Claw closed
                    robot.setServoPositionLeftHand(MecanumRobot.defaultLeftPosition);
                    robot.setServoPositionRightHand(MecanumRobot.defaultRightPosition);
                    autoArmDownState = 3;
                    break;
                case 3:
                    robot.runToPositionSlide(0, -0.5);
                    robot.runWithoutEncoderSlide();
                    autoArmDownState = 4;
                    break;
                case 4:
                    robot.runToPositionArm(0,-0.3);
                    robot.runWithoutEncoderArm();
                    autoArmDownState = 0;
                    break;
            }

            // Auto arm up to release pixels
            switch (autoArmUpState) {
                case 0:
                    // do nothing
                    break;
                case 1:
                    // Wrist up
                    robot.runToPositionArm(MecanumRobot.dropPixelArmPosition,0.5);
                    robot.runWithoutEncoderArm();
                    autoArmUpState = 2;
                    break;
                case 2:
                    robot.runToPositionSlide(MecanumRobot.slideMax, 0.5);
                    robot.runWithoutEncoderSlide();
                    autoArmUpState = 3;
                    break;
                case 3:
                    robot.setServoPositionWrist(MecanumRobot.dropPixelWristPosition);
                    autoArmUpState = 0;
                    break;
            }
            */

            /*
            if (gamepad1.right_bumper) {
                // Whenever the arm thread needs the drivetrain,
                // it needs to interrupt and pause the drivetrain
                driveThread.interrupt();
                robot.AutoLinePark(true);
                driveThread.run();
                telemetry.addData("leaving right bumper",0);
            }
            else if (gamepad1.left_bumper) {
                // Whenever the arm thread needs the drivetrain,
                // it needs to interrupt and pause the drivetrain
                driveThread.interrupt();
                robot.AutoLinePark(false);
                driveThread.run();
                telemetry.addData("leaving left bumper",0);
            }
            */
            /*
            if ((gamepad1.right_bumper) && (gamepad1.left_bumper)) {
                robot.AutoLinePark();
            }
            */

            /* Auto mode
            * April Tag
             */
            /*
            // By Kush
            if (gamepad1.right_bumper) {
                aprilTagRunning = true;
            }
            if (aprilTagRunning) {
                if (gamepad1.x) {
                    checkForRed = true;
                }
                if (gamepad1.y) {
                    checkForBlue = true;
                }
                if (aprilTagDetected && aprilTagMode == 0) {
                    aprilTagMode = 1;
                } else if (aprilTagDetected && aprilTagMode == 1) {
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
                } else if (!aprilTagDetected && aprilTagMode == 0) {
                    if (alliance == 0) {
                        robot.move(1, 0, 0, 0.3);
                    } else if (alliance == 1) {
                        robot.move(-1, 0, 0, 0.3);
                    }
                }
                if (checkForRed) {
                    if (red < robot.getDefaultRed() + 500) {
                        robot.move(0, 1, 0, 0.3);
                    } else {
                        checkForRed = false;
                    }
                }

                if (checkForBlue) {
                    if (blue < robot.getDefaultBlue() + 500) {
                        robot.move(0, 1, 0, 0.3);
                    } else {
                        checkForBlue = false;
                    }
                }

                telemetry.addData("Red: ", red);
                telemetry.addData("Blue: ", blue);
                telemetry.addData("Checking for Red: ", checkForRed);
                telemetry.addData("Checking for Blue: ", checkForBlue);
                telemetry.addData("distance", distance);
            }
            */
            telemetry.update();
        }
        // stop the driving thread.
        driveThread.interrupt();
    }
}




