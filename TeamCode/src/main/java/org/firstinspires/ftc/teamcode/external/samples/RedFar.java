/*
package org.firstinspires.ftc.teamcode;



import static java.lang.Math.abs;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SignalDetectionPipeline;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "RedFar")
public class RedFar extends LinearOpMode {
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    FtcDashboard dashboard = FtcDashboard.getInstance();
    int signal = 0;

    double debugValue = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot();
        robot.initialize(hardwareMap, telemetry);

        int red = robot.colorSensor.red();
        boolean checkForRed = true;
        OpenCvWebcam webcam;
        SignalDetectionPipeline signalDetectionPipeline;


        while (opModeInInit()) {
            telemetry.update();
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalDetectionPipeline = new SignalDetectionPipeline();
        webcam.setPipeline(signalDetectionPipeline); // start camera processFrame()
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start showing the camera
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {

            signal = signalDetectionPipeline.getCounter();
            telemetry.addData("Signal", signal);
            //telemetry.update();
            debugValue = signalDetectionPipeline.getMeanCenter();
            telemetry.addData("meanCenter", debugValue);
            debugValue = signalDetectionPipeline.getMeanLeft();
            telemetry.addData("meanLeft", debugValue);
            debugValue = signalDetectionPipeline.getMeanRight();
            telemetry.addData("meanRight", debugValue);
            telemetry.update();

        }


        while (checkForRed) {
            //red = robot.colorSensor.red();
            if (red >= robot.default_red + 500) {
                robot.move(0, 0, 0, 0);
                checkForRed = false;
            }
            telemetry.addData("Red: ", red);
            telemetry.addData("initial red: ", robot.default_red);
            telemetry.update();
            sleep(10);
        }

    }

}
*/
