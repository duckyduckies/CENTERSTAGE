package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
@Autonomous(name="TeamProdDetectionBlue", group="Tutorials")

public class TeamProdDetectionBlue extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX    = 0.0;
    public static double borderRightX   = 0.0;
    public static double borderTopY     = 0.0;
    public static double borderBottomY  = 0.0;

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Blue Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 50.0, 160.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 255.0);

    boolean checkForBlue = true;

    int targetAprilTag = 2;

    int aprilTagMode = 0;

    public VisionPortal visionPortal;
    public AprilTagProcessor tagProcessor;

    boolean aprilTagRunning = true;

    boolean aprilTagDetected = true;

    double distance = 1000;

    @Override
    public void runOpMode()
    {
        MecanumRobot robot = new MecanumRobot();
        robot.initialize(hardwareMap,telemetry);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        SignalDetectionPipeline myPipeline;
        webcam.setPipeline(myPipeline = new SignalDetectionPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        telemetry.update();


        //

        while (opModeInInit())
        {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if(myPipeline.getRectArea() > 2000){
                if(myPipeline.getRectMidpointX() > 400){
                    AUTONOMOUS_C(); //right
                    robot.RotateP90();
                    sleep(200);
                    robot.move(0,1,0,0.2);
                    sleep(400);
                    // release one pixel
                    robot.move(0,-1,0,0.3);
                    sleep(400);
                }
                else if(myPipeline.getRectMidpointX() > 200){
                    AUTONOMOUS_B(); //center
                    robot.move(0,1,0,0.3);
                    sleep(400);

                    // release one pixel


                    robot.move(0, -1, 0, 0.5);
                    sleep(400);
                    robot.RotateP90();
                }
                else {
                    AUTONOMOUS_A(); //left
                    robot.RotateM90();
                    sleep(200);
                    robot.move(0,1,0,0.2);
                    sleep(400);

                    // release one pixel


                    robot.move(0, -1,0, 0.2);
                    sleep(800);

                }
            }
            telemetry.update();
        }

        waitForStart();

           // webcam.stopStreaming();
           webcam.closeCameraDevice();
            sleep(1000);

            // crash test
            tagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor)
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(640, 480))
                    .build();


        while (aprilTagRunning && opModeIsActive()) {

            aprilTagDetected = false;
            AprilTagDetection myAprilTagDetection = tryDetectApriTag(targetAprilTag);


            if (myAprilTagDetection != null)
            {
                telemetry.addData("April Tag detected: ", "!");
                distance = myAprilTagDetection.ftcPose.y;
                aprilTagDetected = true;
            }

            if (aprilTagDetected && aprilTagMode == 0) {
                aprilTagMode = 1;
            }
            else if (aprilTagDetected && aprilTagMode == 1) {
                double difference = distance ;
                // estimating that it takes 170 ms for robot to move 1 inch forward (power 0.15)

                telemetry.addData("distance =  ", distance);
            }

        }


        telemetry.addData("finish running: ", "ok");
        telemetry.update();

    }

    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    public void AUTONOMOUS_A(){
        telemetry.addLine("Autonomous A");

    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B");

    }
    public void AUTONOMOUS_C(){
        telemetry.addLine("Autonomous C");
    }

    public AprilTagDetection tryDetectApriTag(int idCode)
    {
        AprilTagDetection aprilTagDetection = null;
        List<AprilTagDetection> myAprilTagDetections = tagProcessor.getDetections();
        for (int i = 0; i < myAprilTagDetections.size(); i++) {
            AprilTagDetection myAprilTagDetection = myAprilTagDetections.get(i);

            if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                int myAprilTagIdCode = myAprilTagDetection.id;
                if (myAprilTagIdCode == idCode) {
                    aprilTagDetection = myAprilTagDetection;
                    telemetry.addData("y", myAprilTagDetection.ftcPose.y);
                    telemetry.addData("z", myAprilTagDetection.ftcPose.z);
                    telemetry.addData("roll", myAprilTagDetection.ftcPose.roll);
                    telemetry.addData("pitch", myAprilTagDetection.ftcPose.pitch);
                    telemetry.addData("yaw", myAprilTagDetection.ftcPose.yaw);
                }
            }
        }
        return aprilTagDetection;
    }
}
