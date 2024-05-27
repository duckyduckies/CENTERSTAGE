package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Redclose")
public class Redclose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot();
        robot.initialize(hardwareMap,telemetry);

        boolean aprilTagDetected = false;
        int aprilTagMode = 0;
        int targetAprilTag = 4;
        int alliance = 1;
        int red = robot.colorSensor.red();
        double desiredDistance = 7;
        double distance = Double.MAX_VALUE;
        boolean aprilTagRunning = true;
        boolean checkForRed = true;

        while (opModeInInit())
        {
            telemetry.update();
        }
        //move robot to red lines

        robot.move(0,1,0,0.4);
        sleep(1000);
        robot.move(0,0,0,0);
        sleep(1000);
        //move robot back

        robot.move(0,-1,0,0.4);
        sleep(900);
        robot.move(0,0,0,0);
        sleep(200);
        //turn to face backdrop

        robot.move(0,0,1,0.4);
        sleep(1150);
        robot.move(0,0,0,0);
        //Move robot forward until it senses red

        robot.move(0,1,0,0.4);
        sleep(1000);
        robot.move(0,1,0,0.2);
        while (checkForRed) {
            red = robot.colorSensor.red();
            if(red >= robot.default_red + 500) {
                robot.move(0,0,0,0);
                checkForRed = false;
            }
            telemetry.addData("Red: ", red);
            telemetry.addData("initial red: ", robot.default_red);
            telemetry.update();
            sleep(10);
        }
        //start scanning for april tag

        // april tag start
        if (alliance == 0)
        {
            robot.move(1,0,0,0.2);
        }
        else if (alliance == 1)
        {
            robot.move(-1,0,0,0.2);
        }
        while (aprilTagRunning && opModeIsActive()) {

            aprilTagDetected = false;
            AprilTagDetection myAprilTagDetection = robot.tryDetectApriTag(targetAprilTag);
            telemetry.addData("April Tag detected: ", robot.tagProcessor.getDetections().size());

            if (myAprilTagDetection != null)
            {
                distance = myAprilTagDetection.ftcPose.y;
                aprilTagDetected = true;
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
                aprilTagMode = 0;

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
        robot.move(0,0,-1,0.4);
        sleep(2400);
    }
}
