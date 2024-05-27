package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.utility.Globalvalues;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config //Disable if not using FTC Dashboard https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022#opencv_freightfrenzy_2021-2022
@Autonomous(name="NewAutoBlueFar", group="Tutorials")

public class NewAutoBlueFar extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    int liftMax = 550;
    int wristMax = 550;
    double leftArmPosition;

    boolean checkForRed = true;

    int blue, blue2;

    // blue Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 50.0, 160.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 255.0);

    // Yellow Range
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);

    @Override
    public void runOpMode()
    {
        MecanumRobot robot = new MecanumRobot();
        robot.initialize(hardwareMap,telemetry);


        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));
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
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }


            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if(myPipeline.getRectArea() > 2000){
                if(myPipeline.getRectMidpointX() > 400){   //right
                    AUTONOMOUS_C();
                    //move robot to red lines and stop when detected

                    robot.move(0,1,0,0.4);
                    sleep(1600);
                    robot.move(0,1,0,0.2);

                    checkForRed = true;
                    ElapsedTime elapsedTime = new ElapsedTime();
                    elapsedTime.reset();

                    while (checkForRed && elapsedTime.seconds() < 4) {
                        blue = robot.colorSensor.blue();
                        blue2 = robot.colorSensor2.blue();
                        if (blue >= robot.default_blue + 500 || blue2 >= robot.default_blue + 500)
                        {
                            robot.move(0,0,0,0);
                            checkForRed = false;
                        }


                    }

                    robot.RotateP90();
                    robot.move(0,1,0,0.2);
                    sleep(400);
                    // release one pixel
                    robot.move(0,-1,0,0.3);
                    sleep(400);
                    robot.move(1,0,0,0.6);
                    sleep(800);

                    robot.move(0,0,0,0);
                }
                else if(myPipeline.getRectMidpointX() > 200){  // center
                    AUTONOMOUS_B();
                    //move robot to red lines and stop when detected

                    robot.move(0,1,0,0.4);
                    sleep(1600);
                    robot.move(0,1,0,0.2);

                    checkForRed = true;
                    ElapsedTime elapsedTime = new ElapsedTime();
                    elapsedTime.reset();

                    while (checkForRed && elapsedTime.seconds() < 4) {
                        blue = robot.colorSensor.blue();
                        blue2 = robot.colorSensor2.blue();
                        if (blue >= robot.default_blue + 500 || blue2 >= robot.default_blue + 500)
                        {
                            robot.move(0,0,0,0);
                            checkForRed = false;
                        }


                    }

                    robot.move(0,1,0,0.3);
                    sleep(400);

                    // release one pixel

                    robot.move(0, -1, 0, 0.5);
                    sleep(400);
                    robot.RotateP90();

                    robot.move(0,0,0,0);
                }
                else {                           // left
                    AUTONOMOUS_A();
                    //move robot to red lines and stop when detected

                    robot.move(0,1,0,0.4);
                    sleep(1600);
                    robot.move(0,1,0,0.2);

                    checkForRed = true;
                    ElapsedTime elapsedTime = new ElapsedTime();
                    elapsedTime.reset();

                    while (checkForRed && elapsedTime.seconds() < 4) {
                        blue = robot.colorSensor.blue();
                        blue2 = robot.colorSensor2.blue();
                        if (blue >= robot.default_blue + 500 || blue2 >= robot.default_blue + 500)
                        {
                            robot.move(0,0,0,0);
                            checkForRed = false;
                        }


                    }

                    robot.RotateM90();
                    sleep(200);
                    robot.move(0,1,0,0.2);
                    sleep(400);

                    // release one pixel


                    robot.move(0, -1,0, 0.2);
                    sleep(800);
                    robot.RotateP90();
                    robot.RotateP90();

                    robot.move(0,0,0,0);
                }
            }
        }
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
}