package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "RedClose")
public class RedClose extends LinearOpMode {

    MecanumRobot robot = new MecanumRobot(this);
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot(this);
        robot.initialize();
        boolean debugMode = true;
        boolean aprilTagDetected = false;
        int aprilTagMode = 0;
        int targetAprilTag = 5;
        int alliance = 1;
        int red = robot.getColorSensorRed();
        int i = 0;
        double desiredDistance = 4;
        double distance = Double.MAX_VALUE;
        boolean aprilTagRunning = true;
        boolean checkForRed = true;

        int teamPropLocation = 0;

        while (opModeInInit())
        {
            telemetry.addData("Left Distance Sensor", String.format("%.01f cm", robot.distanceSensorL.getDistance(DistanceUnit.CM)));
            telemetry.addData("Right Distance Sensor", String.format("%.01f cm", robot.distanceSensorR.getDistance(DistanceUnit.CM)));
            telemetry.addData("Left Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawL.getDistance(DistanceUnit.CM)));
            telemetry.addData("Right Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawR.getDistance(DistanceUnit.CM)));

            telemetry.update();
        }
        robot.move(0,1,0,0.4);
        sleep(1000);
        robot.move(0,0,0,0);
        sleep(500);
        //move robot to center spike mark
        robot.move(0,1,0,0.2);

        // object detection
        for (i=0; i<50; i++) {
            sleep(100);
            if (robot.distanceSensorL.getDistance(DistanceUnit.CM) < 10) {
                //Left
                robot.move(1,0,0,0.2);
                sleep(1000);
                robot.move(0,0,0,0);
                robot.move(0,0,-1,0.4);
                sleep(1300);
                robot.move(0,0,0,0);
                robot.move(0,1,0,0.1);
                sleep(300);
                robot.move(1,0,0,0.2);
                sleep(400);
                robot.move(0,0,0,0);
                robot.runToPositionArm(202,0.3);
                sleep(200);
                robot.setServoPositionWrist(0.85);
                sleep(1000);
                robot.setServoPositionLeftHand(1);
                sleep(1000);
                robot.move(0,1,0,0.3);
                sleep(50);
                robot.move(0,0,1,0.4);
                sleep(1300);
                targetAprilTag = 4;
                break;
            }
            else if (robot.distanceSensorR.getDistance(DistanceUnit.CM) < 10) {
                //Right
                robot.move(-1,0,0,0.2);
                sleep(1000);
                robot.move(0,0,0,0);
                robot.move(0,0,1,0.4);
                sleep(1300);
                robot.move(0,0,0,0);
                robot.move(0,1,0,0.1);
                sleep(300);
                robot.move(0,0,0,0);
                robot.runToPositionArm(202,0.3);
                sleep(200);
                robot.setServoPositionWrist(0.85);
                sleep(1000);
                robot.setServoPositionLeftHand(1);
                sleep(1000);
                robot.move(0,1,0,0.3);
                sleep(50);
                robot.move(0,0,-1,0.4);
                sleep(1300);
                targetAprilTag = 6;
                break;
            }
            else if (robot.distanceSensorClawR.getDistance(DistanceUnit.CM) < 19) {
                //Center
                robot.move(0,0,0,0);
                robot.runToPositionArm(202,0.3);
                sleep(400);
                robot.move(0,-1,0,0.2);
                sleep(300);
                robot.move(0, 0, 0, 0);
                robot.setServoPositionWrist(0.85);
                sleep(400);
                robot.move(1,0,0,0.2);
                sleep(400);
                robot.move(0,0,0,0);
                sleep(200);

                robot.setServoPositionLeftHand(1);
                sleep(800);
                targetAprilTag = 5;
                break;
            }
            if (debugMode == true) {
                telemetry.addData("Left Distance Sensor", String.format("%.01f cm", robot.distanceSensorL.getDistance(DistanceUnit.CM)));
                telemetry.addData("Right Distance Sensor", String.format("%.01f cm", robot.distanceSensorR.getDistance(DistanceUnit.CM)));
                telemetry.addData("Left Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawL.getDistance(DistanceUnit.CM)));
                telemetry.addData("Right Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawR.getDistance(DistanceUnit.CM)));
            }
            telemetry.update();
        }
        // once detected, stop the robot
        robot.move(0,-1,0,0.3);
        robot.AutoWristUp();
        sleep(300);
        robot.move(0,0,0,0);

        //turn to face backdrop
        robot.move(0,0,1,0.4);
        sleep(1350);
        robot.move(0,0,0,0);
        //Move robot forward until it senses red

        robot.move(0,1,0,0.4);
        sleep(1400); // move forward using power 0.4 for 1 second
        robot.move(0,0,0,0);
        sleep(100);
        robot.move(0,1,0,0.15);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        boolean redDetected = false;
        while (checkForRed && runtime.milliseconds()<4000) { // move forward using power 0.2 until red line is detected
            red = robot.getColorSensorRed();
            telemetry.addData("Red: ", red);
            telemetry.addData("Red Threshold: ", MecanumRobot.red_threshold);
            telemetry.update();
            if(red >= robot.getDefaultRed()+MecanumRobot.red_diff) { // detects red line
                robot.move(0,0,0,0); // brakes
                checkForRed = false; // will break the while loop
                redDetected = true;
            }

            sleep(10);
        }
        //start scanning for april tag
        telemetry.addData("target tag" , targetAprilTag);
        telemetry.update();

        if (redDetected == true)
        {
            // april tag start
            if (alliance == 0)
            {
                robot.move(1,0,0,0.20);
            }
            else if (alliance == 1)
            {
                robot.move(-1,0,0,0.20);
            }
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();

            while (aprilTagRunning && opModeIsActive() && elapsedTime.milliseconds() < (6000 + (6 - targetAprilTag) * 1500)) {

                aprilTagDetected = false;
                AprilTagDetection myAprilTagDetection = robot.tryDetectAprilTag(targetAprilTag);
                telemetry.addData("April Tag detected: ", robot.getDetectionSize());
                telemetry.addData("target tag" , targetAprilTag);
                if (myAprilTagDetection != null)
                {
                    distance = myAprilTagDetection.ftcPose.y;
                    aprilTagDetected = true;
                    telemetry.addData("distance", distance);
                    telemetry.addLine("target april tag detected");
                }

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
                    aprilTagMode = 2;

                    if (alliance == 0) {
                        robot.move(1, 0, 0, 0.3);
                        sleep(750);
                    } else if (alliance == 1) {
                        robot.move(1, 0, 0, 0.3);
                        sleep(300);
                    }

                    aprilTagRunning = false;
                }
                telemetry.update();
                sleep(10);
            }
            robot.move(0,0, 0, 0);
            if (aprilTagMode == 2)
            {
                robot.AutoArmUp();

                robot.move(0,1,0,0.4);
                sleep(550);
                robot.move(0, 0, 0, 0);
                sleep(500);
                robot.setServoPositionLeftHand(0.5);
                robot.setServoPositionRightHand(0.5);
                sleep(400);

                robot.move(0, -1, 0, 0.3);
                sleep(200);
                robot.move(0, 0, 0, 0);
                robot.AutoArmDown();

                if (alliance == 0)
                {
                    robot.move(1,0,0,0.25);
                }
                else if (alliance == 1)
                {
                    robot.move(-1,0,0,0.25);
                }
                sleep(500);
                robot.move(0, 0, 0, 0);
            }
        }
    }
}
