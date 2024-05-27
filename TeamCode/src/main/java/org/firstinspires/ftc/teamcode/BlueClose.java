package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "BlueClose")
public class BlueClose extends LinearOpMode {

    MecanumRobot robot = new MecanumRobot(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize();
        boolean debugMode = true;
        boolean aprilTagDetected = false;
        int aprilTagMode = 0;
        int targetAprilTag = 2;
        int alliance = 0;
        int blue = robot.getColorSensorBlue();
        int i = 0;
        double desiredDistance = 4;
        double distance = Double.MAX_VALUE;
        boolean aprilTagRunning = true;
        boolean checkForBlue = true;

        int teamPropLocation = 0;

        while (opModeInInit())
        {
            telemetry.addData("Left Distance Sensor", String.format("%.01f cm", robot.distanceSensorL.getDistance(DistanceUnit.CM)));
            telemetry.addData("Right Distance Sensor", String.format("%.01f cm", robot.distanceSensorR.getDistance(DistanceUnit.CM)));
            telemetry.addData("Left Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawL.getDistance(DistanceUnit.CM)));
            telemetry.addData("Right Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawR.getDistance(DistanceUnit.CM)));

            telemetry.update();
        }
        //move robot to center spike mark
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
                targetAprilTag = 1;
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
                targetAprilTag = 3;
                break;
            }
            else if (robot.distanceSensorClawR.getDistance(DistanceUnit.CM) < 19)
            {
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

                targetAprilTag = 2;
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
        robot.move(0,0,-1,0.4);
        sleep(1350);
        robot.move(0,0,0,0);
        //Move robot forward until it senses blue

        robot.move(0,1,0,0.4);
        sleep(1500); // move forward using power 0.4 for 1 second
        robot.move(0,1,0,0.12);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        boolean blueDetected = false;
        while (checkForBlue && runtime.milliseconds()<7000) { // move forward using power 0.2 until blue line is detected
            blue = robot.getColorSensorBlue();
            // default blue value in the gray (178)
            // blue value in the blue line (245...increased by 67)
            if(blue >= robot.getDefaultBlue() + MecanumRobot.blue_diff) { // detects blue line
                robot.move(0,0,0,0); // brakes
                checkForBlue = false; // will break the while loop\
                blueDetected = true;
            }
            telemetry.addData("Blue: ", blue);
            telemetry.addData("Blue threshold: ", MecanumRobot.blue_threshold);
            telemetry.update();
            sleep(10);
        }
        //start scanning for april tag
        telemetry.addData("target tag" , targetAprilTag);
        telemetry.update();
        if (blueDetected == true)
        {
            // april tag start
            if (alliance == 0)
            {
                robot.move(1,0,0,0.2);
            }
            else if (alliance == 1)
            {
                robot.move(-1,0,0,0.2);
            }

            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            while (aprilTagRunning && opModeIsActive() && elapsedTime.milliseconds() < (6000 + (targetAprilTag - 1) * 1500)) {
                aprilTagDetected = false;
                AprilTagDetection myAprilTagDetection = robot.tryDetectAprilTag(targetAprilTag);
                telemetry.addData("target tag" , targetAprilTag);
                telemetry.addData("April Tag detected: ", robot.getDetectionSize());

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
