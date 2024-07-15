package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "FarClose")
public class FarCode extends LinearOpMode {

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
            telemetry.addData("Middle Distance Sensor", String.format("%.01f cm", robot.distanceSensorMiddle.getDistance(DistanceUnit.CM)));

            telemetry.update();
        }
        robot.move(0,1,0,0.4);
        sleep(1150);
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
                robot.setServoPositionLeftHand(0);
                robot.setServoPositionWrist(0);
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
                robot.move(0,1,0,0.2);
                sleep(300);
                robot.move(0,0,0,0);
                robot.setServoPositionLeftHand(1);
                sleep(400);
                robot.setServoPositionLeftHand(0);
                robot.setServoPositionWrist(0);
                sleep(1000);
                robot.move(0,0,-1,0.4);
                sleep(1300);
                robot.move(0,0,0,0);
                targetAprilTag = 6;
                break;
            }
            else if (robot.distanceSensorMiddle.getDistance(DistanceUnit.CM) < 19) {
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
                robot.setServoPositionLeftHand(0);
                robot.setServoPositionWrist(0);
                targetAprilTag = 5;
                break;
            }
            if (debugMode == true) {
                telemetry.addData("Left Distance Sensor", String.format("%.01f cm", robot.distanceSensorL.getDistance(DistanceUnit.CM)));
                telemetry.addData("Right Distance Sensor", String.format("%.01f cm", robot.distanceSensorR.getDistance(DistanceUnit.CM)));
                telemetry.addData("Middle Distance Sensor", String.format("%.01f cm", robot.distanceSensorMiddle.getDistance(DistanceUnit.CM)));
            }
            telemetry.update();
        }
    }
}
