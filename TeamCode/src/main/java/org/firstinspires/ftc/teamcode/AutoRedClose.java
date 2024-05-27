package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.teamTeleOpCode.wristPosition;
import static java.lang.Math.abs;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;




@Autonomous(name = "AutoRedClose")
public class AutoRedClose extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public VisionPortal visionPortal;

    public AprilTagProcessor tagProcessor;

    public static double borderLeftX    = 0.0;
    public static double borderRightX   = 0.0;
    public static double borderTopY     = 0.0;
    public static double borderBottomY  = 0.0;

    int liftMax = 550;
    int wristMax = 550;
    double leftArmPosition;


    // Red Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);


    public int moveDirection = 1;//0=left, 1=center, 2=right
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot();
        robot.initialize(hardwareMap,telemetry);

        boolean aprilTagDetected = false;
        int aprilTagMode = 0;
        int targetAprilTag = 4;
        int alliance = 1;
        int red, red2;
        double desiredDistance = 7;
        double distance = Double.MAX_VALUE;
        boolean aprilTagRunning = true;
        boolean checkForRed = true;

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


        while (opModeInInit())
        {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }

       //     telemetry.addData("RectArea: ", myPipeline.getRectArea());
       //     telemetry.update();

            if(myPipeline.getRectArea() > 2000){
                if(myPipeline.getRectMidpointX() > 400){
                    //AUTONOMOUS_C(); //right
                    moveDirection = 2;
                    targetAprilTag = 6;
                    telemetry.addData("object on left: ", moveDirection);
         //           telemetry.update();
                }
                else if(myPipeline.getRectMidpointX() > 200){
                    //AUTONOMOUS_B(); //center
                    moveDirection = 1;
                    targetAprilTag = 5;
                    telemetry.addData("object on center: ", moveDirection);
           //         telemetry.update();
                }
                else {
                    //AUTONOMOUS_A(); //left
                    moveDirection = 0;
                    targetAprilTag = 4;
                    telemetry.addData("object on right: ", moveDirection);
            //        telemetry.update();
                }
                telemetry.update();
            }

        }

        waitForStart();

        // webcam.stopStreaming();
        webcam.closeCameraDevice();
        sleep(1000);

        robot.intializeAprilTag(hardwareMap);



        //move robot to red lines and stop when detected

        robot.move(0,1,0,0.4);
        sleep(1200);
        robot.move(0,1,0,0.2);

        checkForRed = true;
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (checkForRed && elapsedTime.seconds() < 4) {
            red = robot.colorSensor.red();
            red2 = robot.colorSensor2.red();
            if (red >= robot.default_red + 500 || red2 >= robot.default_red + 500)
            {
                robot.move(0,0,0,0);
                checkForRed = false;
            }


        }



        if (moveDirection == 0){  //Object is located on left side
            telemetry.addData("object on: ", moveDirection);
            telemetry.update();
            robot.RotateM90();
            sleep(200);
            robot.move(0,1,0,0.2);
            sleep(400);

            // release one pixel


            robot.move(0, -1,0, 0.2);
            sleep(800);
            robot.RotateP90();
            robot.RotateP90();

        }else if(moveDirection == 1){   //Object is located on center
            //sleep(1000);
            telemetry.addData("object on: ", moveDirection);
            telemetry.update();
            robot.move(0,1,0,0.3);
            sleep(400);

            // release one pixel


            robot.move(0, -1, 0, 0.5);
            sleep(400);
            robot.RotateP90();
        } else //object on right
        {
            telemetry.addData("object on: ", moveDirection);
            telemetry.update();
            robot.RotateP90();
            sleep(200);
            robot.move(0,1,0,0.2);
            sleep(400);
            // release one pixel
            robot.move(0,-1,0,0.3);
            sleep(400);
            robot.move(1,0,0,0.6);
            sleep(800);



        }


        //Move robot forward until it senses red
        checkForRed = true;
        robot.move(0,1,0,0.5);
        sleep(1000);
        robot.move(0,1,0,0.2);
        elapsedTime.reset();
        while (checkForRed && elapsedTime.seconds() < 3) {
            red = robot.colorSensor.red();
            red2 = robot.colorSensor2.red();
            if (red >= robot.default_red + 300 || red2 >= robot.default_red + 300) {
                robot.move(0,0,0,0);
                checkForRed = false;
            }
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
                    robot.move(0, 1, 0, 0.2);
/////////////////////////going up

                    sleep((long) (170 * difference));
                } else if (difference < -0.1) {
                    robot.move(0, -1, 0, 0.2);
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
//        robot.move(0,0,-1,0.4);
//        sleep(2400);
        robot.RotateP90();
        robot.RotateP90();
        robot.move(0, -1, 0, 0.3);
        sleep(350);

        while(leftArmPosition >=liftMax) {
            robot.motor1ex.setPower(0.6);
            robot.motor2ex.setPower(0.6);
            leftArmPosition = robot.motor1ex.getCurrentPosition();
        }

        //wristPosition = Constants.wristUp;

        sleep(500);
        robot.servo2.setPosition(Constants.clawOpen);
        robot.servo3.setPosition(Constants.clawOpen);
        sleep(200);
        robot.move(0, 1, 0, 0.3);


    }
}
