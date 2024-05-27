package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;


// Autonomous mode
//import com.google.blocks.ftcrobotcontroller.runtime.AprilTagAccess;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class MecanumRobot {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor motorHDLeftFront = null;
    private DcMotor motorHDLeftRear = null;
    private DcMotor motorHDRightFront = null;
    private DcMotor motorHDRightRear = null;
    private DcMotor motorLeftArm = null;
    private DcMotor motorRightArm = null;
    private DcMotor motorSlides = null;

    public final static int slideMax = 245;
    public final static int slideMin = 0;
    public final static int armMax = 2900;
    public final static int armMin = 0;
    
    public final static int autoArmUpArm = 700;
    public final static int autoArmUpBackArm = 2793; //2397;
    public final static DcMotorSimple.Direction defaultDirectionLeftArm = DcMotorSimple.Direction.FORWARD;
    public final static DcMotorSimple.Direction defaultDirectionRightArm = DcMotorSimple.Direction.FORWARD;
    public final static DcMotorSimple.Direction defaultDirectionSlide = DcMotorSimple.Direction.FORWARD;

    public final static int autoArmUpBackSlide = 186; //0;
    private Servo servoWrist = null;
    private Servo servoLeftHand = null;
    private Servo servoRightHand = null;
    private Servo servoLauncher = null;

    public final static int defaultLeftPosition = 0; // claw closed
    public final static int defaultRightPosition = 1; // claw closed
    public final static int defaultWristPosition = 0; // wrist up
    public final static int wristUp = 0;
    public final static int wristDown = 1;
    public final static double autoArmUpBackWrist = 0.35; //0.2;

    public final static double autoArmUpWrist = 0.6;

    public final static int slidePickupArm = 135;

    public final static int defaultLauncherPosition = 0;


    public TouchSensor touchSensor = null;
    public DistanceSensor distanceSensorL = null;
    public DistanceSensor distanceSensorR = null;
    public DistanceSensor distanceSensorClawL = null;
    public DistanceSensor distanceSensorClawR = null;

    private ColorSensor colorSensor, colorSensorL;

    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 10;

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;

    RevBlinkinLedDriver blinkinLedDriver;
    public final static RevBlinkinLedDriver.BlinkinPattern defaultPattern = RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY;
    public final static RevBlinkinLedDriver.BlinkinPattern greenPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    public final static RevBlinkinLedDriver.BlinkinPattern yellowPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    public final static RevBlinkinLedDriver.BlinkinPattern redPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
    private String currentPattern;
    private MecanumRobot.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;
    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    // Auto mode
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    private IMU imu;

    public int default_red;
    public int default_blue;
    public int default_red_left;
    public int default_blue_left;

    public final static int red_diff = 700;
    public final static int blue_diff = 1800;
    public final static int red_diff_left = 250;
    public final static int blue_diff_left = 1200;
    public final static int red_threshold = 2800; // compared to 1900
    public final static int blue_threshold = 5400; // compared to 3000
    public final static int red_threshold_left = 1300; // compared to 1000
    public final static int blue_threshold_left = 3800; // compared to 2100
    private boolean debugMode=true;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public MecanumRobot (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void initialize()
    {
        // Mecanum Drivertrain Motors

        motorHDLeftFront = myOpMode.hardwareMap.get(DcMotor.class, "HDMotor2");
        motorHDLeftRear = myOpMode.hardwareMap.get(DcMotor.class, "HDMotor3");
        motorHDRightFront = myOpMode.hardwareMap.get(DcMotor.class, "HDMotor0");
        motorHDRightRear = myOpMode.hardwareMap.get(DcMotor.class, "HDMotor1");

        // RUN_WITHOUT_ENCODER mode - only set direction & power

        motorHDLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHDLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHDRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHDRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorHDLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorHDLeftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHDRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHDRightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // Top Motors

        motorLeftArm = myOpMode.hardwareMap.get(DcMotor.class, "HDMotorArmL");
        motorRightArm = myOpMode.hardwareMap.get(DcMotor.class, "HDMotorArmR");
        motorSlides = myOpMode.hardwareMap.get(DcMotor.class, "CoreMotorSlide");

        // If with external encoder:
        // try both directions and choose the one that results in positive encoder ticks
        motorLeftArm.setDirection(defaultDirectionLeftArm);
        myOpMode.telemetry.addData("direction of left arm motor","reverse");
        motorRightArm.setDirection(defaultDirectionRightArm);
        motorSlides.setDirection(defaultDirectionSlide);
        myOpMode.telemetry.addData("direction of slide motor","forward");

        // Merely brakes the arm when it's raised and stop falling to gravity
        motorLeftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos

        servoWrist = myOpMode.hardwareMap.get(Servo.class, "ServoWrist");
        servoLeftHand = myOpMode.hardwareMap.get(Servo.class, "ServoClawL");
        servoRightHand = myOpMode.hardwareMap.get(Servo.class, "ServoClawR");
        servoLauncher = myOpMode.hardwareMap.get(Servo.class, "ServoLauncher");

        touchSensor = myOpMode.hardwareMap.get(TouchSensor.class, "touchSensor");

        AutoArmDown(); // includes wrist & claw actions
        runWithoutEncoderSlide();
        runWithoutEncoderArm();

        setServoPositionLauncher(defaultLauncherPosition);
        /*
        // Calibrates arm position using touch sensor
        // We don't need this because as long as robot is at 0 position when it's turned on
        // the encoder will remember the position while power is on
        ElapsedTime runtime2 = new ElapsedTime(); // prevent infinite loop
        runtime2.reset();
        setMotorPowerArm(-0.2);
        while (runtime2.seconds()<2) {
            if (touchSensor.isPressed()) {
                myOpMode.telemetry.addData("Touch Sensor", "Is Pressed");
                break;
            }
        }
        setMotorPowerArm(0);
        */

        distanceSensorL = myOpMode.hardwareMap.get(DistanceSensor.class, "distanceSensorL");
        distanceSensorR = myOpMode.hardwareMap.get(DistanceSensor.class, "distanceSensorR");
        distanceSensorClawL = myOpMode.hardwareMap.get(DistanceSensor.class, "distanceSensorClawL");
        distanceSensorClawR = myOpMode.hardwareMap.get(DistanceSensor.class, "distanceSensorClawR");

        /*
        * Auto mode initialization
        */

        /*
        //By Bo
        motor0core.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0core.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1core.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1core.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2core.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2core.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "colorSensorR");
        colorSensor.enableLed(true);
        colorSensorL = myOpMode.hardwareMap.get(ColorSensor.class, "colorSensorL");
        colorSensorL.enableLed(true);

        default_red = colorSensor.red();
        default_blue = colorSensor.blue();
        default_red_left = colorSensorL.red();
        default_blue_left = colorSensorL.blue();

        displayKind = MecanumRobot.DisplayKind.MANUAL;

        blinkinLedDriver = myOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        currentPattern = defaultPattern.toString();
        blinkinLedDriver.setPattern(defaultPattern);

        myOpMode.telemetry.addData("Display Kind: ", displayKind.toString());
        myOpMode.telemetry.addData("Pattern: ", defaultPattern.toString());


        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
        intializeAprilTag();
        /* local variables
        int myAprilTaIdCode = -1;
        int targetAprilTag = 2;
        boolean aprilTagRunning = false;
        // mode 0 : scanning
        // mode 1 : approaching
        int aprilTagMode = 0;
        double desiredDistance = 7;
        boolean aprilTagDetected = false;
        double distance = Double.MAX_VALUE;
        */
    }


    /**
     * Mecanum Drivetrain
     * Calculates the left/right front/rear motor powers required to achieve the requested
     * robot motions: ...
     * Then sends these power levels to the motors.
     *
     * @param x
     * @param y
     * @param turn
     * @param powerScale
     */
    public void move(double x, double y, double turn, double powerScale) {
        double theta = Math.atan2(y,x);
        double power = Math.hypot(x,y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),
                Math.abs(cos));
        double leftFront = power * cos/max + turn;
        double rightFront = power * sin/max - turn;
        double leftRear = power * sin/max + turn;
        double rightRear = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFront /= power + Math.abs(turn);
            rightFront /= power + Math.abs(turn);
            leftFront /= power + Math.abs(turn);
            rightRear /= power + Math.abs(turn);
        }
        motorHDLeftFront.setPower(leftFront * powerScale);
        motorHDLeftRear.setPower(leftRear * powerScale);
        motorHDRightFront.setPower(rightFront * powerScale);
        motorHDRightRear.setPower(rightRear * powerScale);

        /*
        if (debugMode) {
            myOpMode.telemetry.addData("Motor 0 Left Front", leftFront * powerScale);
            myOpMode.telemetry.addData("Motor 1 Left Rear", leftRear * powerScale);
            myOpMode.telemetry.addData("Motor 2 Right Front", rightFront * powerScale);
            myOpMode.telemetry.addData("Motor 3 Right Rear", rightRear * powerScale);
        }
         */
    }

    /**
     * Arm movement
     * Set both motor powers
     * Then sends these power levels to the motors.
     *
     * @param
     */
    public void stopAndResetArmSlide() {
        // Reset zero position
        // sometimes randomly reverses directions when switched to STOP_AND_RESET_ENCODER mode (reverse -> forward and vice versa)
        motorSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runWithoutEncoderArm(){
        // RUN_WITHOUT_ENCODER mode - only set direction & power
        // After using RUN_TO_POSITION or STOP_AND_RESET_ENCODER
        // Must change to the WITHOUT ENCODER mode; otherwise, the core motor won't move when set power
        // Before switching, make sure motors brake
        // (We already brake after each call of runToPosition())

        motorLeftArm.setPower(0);
        motorLeftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightArm.setPower(0);
        motorRightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runWithoutEncoderSlide(){
        // RUN_WITHOUT_ENCODER mode - only set direction & power
        // After using RUN_TO_POSITION or STOP_AND_RESET_ENCODER
        // Must change to the WITHOUT ENCODER mode; otherwise, the core motor won't move when set power
        // Before switching, make sure motors brake
        // (We already brake after each call of runToPosition())

        motorSlides.setPower(0);
        motorSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setMotorPowerArm(double powerScale) {
        motorLeftArm.setPower(powerScale);
        motorRightArm.setPower(powerScale);
    }
    public void runToPositionArm(int position,double power) {
        motorLeftArm.setTargetPosition(position);
        motorRightArm.setTargetPosition(position);
        motorLeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPowerArm(power);
        while (motorLeftArm.isBusy() && motorRightArm.isBusy()) myOpMode.idle();
        setMotorPowerArm(0);
        /*
        // With the external encoder, RUN_TO_POSITION does NOT work
        // Arm down to position 0
        ElapsedTime runtime2 = new ElapsedTime(); // prevent infinite loop
        runtime2.reset();
        setMotorPowerArm(0.2);
        while (motorLeftArm.getCurrentPosition()>0 && runtime2.seconds() < 5);
        setMotorPowerArm(0); // IMPORTANT: brake
         */
    }

    public int getMotorPositionLeftArm(){
        return motorLeftArm.getCurrentPosition();
    }
    public int getMotorPositionRightArm() { return motorRightArm.getCurrentPosition(); }
    /**
     * Slide movement
     *
     * @param powerScale
     */
    public void setMotorPowerSlide(double powerScale) {
        motorSlides.setPower(powerScale);
    }
    public void runToPositionSlide(int position, double power) {

        motorSlides.setTargetPosition(position);
        motorSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlides.setPower(power);
        while (motorSlides.isBusy()) myOpMode.idle();
        motorSlides.setPower(0);
        /*
        // With the external encoder, RUN_TO_POSITION does NOT work
        ElapsedTime runtime = new ElapsedTime(); // prevent infinite loop
        runtime.reset();
        motorSlides.setPower(power);
        if (motorSlides.getCurrentPosition()>position)
            while (motorSlides.getCurrentPosition()>position && runtime.seconds() < 0.5) idle();
        else if (position<motorSlides.getCurrentPosition())
            while (position<motorSlides.getCurrentPosition() && runtime.seconds() < 0.5) idle();
        motorSlides.setPower(0); // IMPORTANT: brake
        */
    }
    public int getMotorPositionSlide(){
        return motorSlides.getCurrentPosition();
    }
    //public DcMotorSimple.Direction getMotorDirectionSlide() { return motorSlides.getDirection(); }

    /**
     * Hand movement
     *
     * @param
     */
    public double getServoPositionLeftHand() {
        return servoLeftHand.getPosition();
    }

    public double getServoPositionRightHand() {

        return servoRightHand.getPosition();
    }
    public void setServoPositionLeftHand(double position) {
        servoLeftHand.setPosition(position);
    }
    public void setServoPositionRightHand(double position) {
        servoRightHand.setPosition(position);
    }
    /**
     * Wrist movement
     * @param
     */
    public double getServoPositionWrist() {
        return servoWrist.getPosition();
    }
    public void setServoPositionWrist(double position) {
        servoWrist.setPosition(position);
    }
    /**
     * Launcher
     * @param
     */
    public void setServoPositionLauncher(double position) {
        servoLauncher.setPosition(position);
    }

    public void AutoArmDown() {
        // Wrist up
        setServoPositionWrist(defaultWristPosition);

        // Claw closed
        setServoPositionLeftHand(defaultLeftPosition);
        setServoPositionRightHand(defaultRightPosition);

        runToPositionSlide(0, -0.5);
        runToPositionArm(0,-0.3);
        // Calibrates arm position using touch sensor
        // We don't need this because as long as robot is at 0 position when it's turned on
        // the encoder will remember the position while power is on
        ElapsedTime runtime2 = new ElapsedTime(); // prevent infinite loop
        runtime2.reset();
        setMotorPowerArm(-0.2);
        while (runtime2.seconds()<0.5) {
            if (touchSensor.isPressed()) {
                myOpMode.telemetry.addData("Touch Sensor", "Is Pressed");
                break;
            }
        }
        setMotorPowerArm(0);
        stopAndResetArmSlide();

    }

    public void AutoArmUp() {
        runToPositionArm(autoArmUpArm,0.5);
        runToPositionSlide(slideMax, 0.5);
        setServoPositionWrist(autoArmUpWrist);
        /*
        //Raises the left arm to 3628 ticks
        ElapsedTime runtime2 = new ElapsedTime(); // prevent infinite loop
        runtime2.reset();
        setMotorPowerArm(0.5);
        while (motorLeftArm.getCurrentPosition()<3629 && runtime2.seconds() < 5) {
            idle();
        }
        setMotorPowerArm(0); // IMPORTANT: brake

        // Extends the slide to slideMax ticks
        ElapsedTime runtime = new ElapsedTime(); // prevent infinite loop
        runtime.reset();
        motorSlides.setPower(0.5);
        while (motorSlides.getCurrentPosition()<10119 && runtime.seconds() < 3) {
            idle();
        }
        motorSlides.setPower(0); // IMPORTANT: brake
        myOpMode.telemetry.addData("slide new position:",motorSlides.getCurrentPosition());

        // Puts down the wrist to position 0.6
        servoWrist.setPosition(0.6);
        */
    }

    public void AutoSlidePickup() {
        runToPositionArm(slidePickupArm,0.5);
        runToPositionSlide(slideMax, 0.5);
        setServoPositionWrist(wristDown);
    }
    public void AutoArmUpBack() {
        runToPositionArm(autoArmUpBackArm, 0.5);
        runToPositionSlide(autoArmUpBackSlide, 0.5);
        setServoPositionWrist(autoArmUpBackWrist);

    }
    public void AutoWristDown() {
        // Opens claws
        setServoPositionLeftHand(1);
        setServoPositionRightHand(0);
        // Puts the wrist down
        servoWrist.setPosition(wristDown);
        //myOpMode.sleep(1500);

    }

    public void AutoWristUp() {
        // Closes claws
        setServoPositionLeftHand(0);
        setServoPositionRightHand(1);
        myOpMode.sleep( 1500);
        // Puts the wrist up
        servoWrist.setPosition(wristUp);
    }

    /*
    * Automatically parks at a red or blue line
    *
    * Happens in the manual mode in front of the backdrop and the human player
    * and also in the auto mode before placing a purple pixel on the ground
    *
    * Algorithm:
    * (Wrong)
    * While under a timer
    * Moves forward until a color sensor detects blue or red
    * Rotates the other side of drivetrain so another color sensor can also meet the line
    *
    * Moves forward until a color sensor detects the line
    * (Be aware the case that robot is already on the line)
    * While under a timer & NOT BOTH color sensors detect the line
    * 1. Rotates a little bit, which will always goes backward, because it rotates around the center
    * 2. Moves forward until a color sensor detects the line

     */
    public void AutoLinePark(boolean findRightOrLeft) {
        boolean leftDetected = false;
        boolean rightDetected = false;

        int blue=0;
        int blueL=0;
        int red=0;
        int redL=0;

        // First check once
        // The robot might already be on the line
        if (findRightOrLeft) {
            blue = getColorSensorBlue();
            red = getColorSensorRed();
        } else {
            blueL = getLeftColorSensorBlue();
            redL = getLeftColorSensorRed();
        }
        if((blue >= getDefaultBlue() + blue_diff) ||  (red >= getDefaultRed() + red_diff)) {
            //if((blue >= blue_threshold) ||  (red >= red_threshold)) {
            rightDetected = true;
            myOpMode.telemetry.addData("RIGHT LINE DETECTED", "");
        }
        if((blueL >= getLeftDefaultBlue() + blue_diff_left) ||  (redL >= getLeftDefaultRed() + red_diff_left)) {
            //if((blueL >= blue_threshold_left) || (redL >= red_threshold_left)) {
            leftDetected = true;
            myOpMode.telemetry.addData("LEFT LINE DETECTED", "");
        }

        myOpMode.telemetry.addData("Right Blue: ", blue);
        myOpMode.telemetry.addData("Right Red: ", red);
        myOpMode.telemetry.addData("Left Blue: ", blueL);
        myOpMode.telemetry.addData("Left Red: ", redL);
        myOpMode.telemetry.addData("Right Blue Default: ", default_blue);
        myOpMode.telemetry.addData("Right Red Default: ", default_red);
        myOpMode.telemetry.addData("Left Blue Default: ", default_blue_left);
        myOpMode.telemetry.addData("Left Red Default: ", default_red_left);

        /*
        myOpMode.telemetry.addData("Right Blue Threshold: ", blue_threshold);
        myOpMode.telemetry.addData("Right Red Threshold: ", red_threshold);
        myOpMode.telemetry.addData("Left Blue Threshold: ", blue_threshold_left);
        myOpMode.telemetry.addData("Left Red Threshold: ", red_threshold_left);
        */
        myOpMode.telemetry.update();

        if ((!rightDetected) && (!leftDetected)) {
            // Moves forward at power 0.2 until a line is detected
            move(0, 1, 0, 0.1);
            while (myOpMode.opModeIsActive() && (!rightDetected) && (!leftDetected)) {
                if (findRightOrLeft) {
                    blue = getColorSensorBlue();
                    red = getColorSensorRed();
                } else {
                    blueL = getLeftColorSensorBlue();
                    redL = getLeftColorSensorRed();
                }

                if((blue >= getDefaultBlue() + blue_diff) ||  (red >= getDefaultRed() + red_diff)) {
                    //if ((blue >= blue_threshold) || (red >= red_threshold)) {
                    rightDetected = true;
                    myOpMode.telemetry.addData("RIGHT LINE DETECTED", "");
                }
                if((blueL >= getLeftDefaultBlue() + blue_diff_left) ||  (redL >= getLeftDefaultRed() + red_diff_left)) {
                    //if ((blueL >= blue_threshold_left) || (redL >= red_threshold_left)) {
                    leftDetected = true;
                    myOpMode.telemetry.addData("LEFT LINE DETECTED", "");
                }
                myOpMode.telemetry.update();
                myOpMode.telemetry.addData("Right Blue: ", blue);
                myOpMode.telemetry.addData("Right Red: ", red);
                myOpMode.telemetry.addData("Left Blue: ", blueL);
                myOpMode.telemetry.addData("Left Red: ", redL);
                myOpMode.telemetry.addData("Right Blue Default: ", default_blue);
                myOpMode.telemetry.addData("Right Red Default: ", default_red);
                myOpMode.telemetry.addData("Left Blue Default: ", default_blue_left);
                myOpMode.telemetry.addData("Left Red Default: ", default_red_left);

                /*
                myOpMode.telemetry.addData("Right Blue Threshold: ", blue_threshold);
                myOpMode.telemetry.addData("Right Red Threshold: ", red_threshold);
                myOpMode.telemetry.addData("Left Blue Threshold: ", blue_threshold_left);
                myOpMode.telemetry.addData("Left Red Threshold: ", red_threshold_left);
                */
                myOpMode.sleep(10);
                //myOpMode.idle();
            }
            move(0, 0, 0, 0);
        }
    }

    public String getPattern() {
        return currentPattern;
    }
    protected void displayPattern(RevBlinkinLedDriver.BlinkinPattern pattern)
    {
        blinkinLedDriver.setPattern(pattern);
        currentPattern = currentPattern.toString();
    }
    public void intializeAprilTag()
    {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public AprilTagDetection tryDetectAprilTag(int idCode)
    {
        AprilTagDetection aprilTagDetection = null;
        List<AprilTagDetection> myAprilTagDetections = tagProcessor.getDetections();
        for (int i = 0; i < myAprilTagDetections.size(); i++) {
            AprilTagDetection myAprilTagDetection = myAprilTagDetections.get(i);

            if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                int myAprilTagIdCode = myAprilTagDetection.id;
                if (myAprilTagIdCode == idCode) {
                    aprilTagDetection = myAprilTagDetection;
                    myOpMode.telemetry.addData("y", myAprilTagDetection.ftcPose.y);
                    myOpMode.telemetry.addData("z", myAprilTagDetection.ftcPose.z);
                    myOpMode.telemetry.addData("roll", myAprilTagDetection.ftcPose.roll);
                    myOpMode.telemetry.addData("pitch", myAprilTagDetection.ftcPose.pitch);
                    myOpMode.telemetry.addData("yaw", myAprilTagDetection.ftcPose.yaw);
                }
            }
        }
        return aprilTagDetection;
    }

    public int getDefaultBlue() { return default_blue; }

    public int getDefaultRed() {
        return default_red;
    }
    public int getLeftDefaultBlue() { return default_blue_left; }

    public int getLeftDefaultRed() {
        return default_red_left;
    }

    public int getColorSensorBlue() {
        return (colorSensor.blue());
    }
    public int getLeftColorSensorBlue() {
        return colorSensorL.blue();
    }
    public int getColorSensorRed() {
        return colorSensor.red();
    }
    public int getLeftColorSensorRed() {
        return colorSensorL.red();
    }

    //public int getColorSensorGreen() { return colorSensor.green(); }

    public int getDetectionSize() {
        return tagProcessor.getDetections().size();
    }

    //public int getColorSensorAlpha() { return colorSensor.alpha(); }

    //public int getColorSensorArgb() { return colorSensor.argb(); }

    /*
    // by Bo
    private void pickUpPixel() {
        clawPosition = Constants.clawOpen;
        servo2.setPosition(clawPosition);
        servo3.setPosition(clawPosition);
        sleep(100);
        wristPosition = Constants.wristDown;
        servo1.setPosition(wristPosition);
        sleep(1000);
        clawPosition = Constants.clawClose;
        servo2.setPosition(clawPosition);
        servo3.setPosition(clawPosition);
        sleep(1000);
        wristPosition = Constants.wristUp;
        servo1.setPosition(wristPosition);
    }

    private void releasePixel() {
        clawPosition = Constants.clawOpen;
        servo2.setPosition(clawPosition);
        servo3.setPosition(clawPosition);
        sleep(100);
        wristPosition = Constants.wristDown;
        servo1.setPosition(wristPosition);
        sleep(1000);
        clawPosition = Constants.clawClose;
        servo2.setPosition(clawPosition);
        servo3.setPosition(clawPosition);
        sleep(1000);
        wristPosition = Constants.wristUp;
        servo1.setPosition(wristPosition);
    }

    private int getLeftArmPosition() {
        return motor0core.getCurrentPosition();
    }

    private int getWristPosition() {
        return motor2core.getCurrentPosition();
    }
    */

    private void RotateP90() {
        imu.resetYaw(); // set to 0 degree
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentAngle = currentAngle *  180 / Math.PI;
        double leftFront, rightFront, leftRear, rightRear, turn;
        double diff =0;
        diff = (90+currentAngle); // needs to move this much: diff
        double sign;
        while (Math.abs(diff)>2){ // while there is more than 2 degree to move
            sign = diff/Math.abs(diff);
            // increase the constant to increase turning speed
            turn = sign*0.2; //turn = sign*0.3; // by Bo
            leftFront =  turn;
            rightFront = -turn;
            leftRear =  turn;
            rightRear =  - turn;

            motorHDLeftFront.setPower(leftFront);
            motorHDLeftRear.setPower(leftRear);
            motorHDRightFront.setPower(rightFront);
            motorHDRightRear.setPower(rightRear);

            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            diff = (90+currentAngle);
            //telemetry.addData("current angle", currentAngle);
            //telemetry.addData("diff", diff);
            //telemetry.update();
        }
        motorHDLeftFront.setPower(0); //brake
        motorHDLeftRear.setPower(0);
        motorHDRightFront.setPower(0);
        motorHDRightRear.setPower(0);
    }

    private void RotateM90() {
        imu.resetYaw(); // set to 0 degree
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentAngle = currentAngle *  180 / Math.PI;
        double leftFront, rightFront, leftRear, rightRear, turn;
        double diff =0;
        diff = (90-currentAngle);
        double sign;
        while (Math.abs(diff)>2){
            sign = diff/Math.abs(diff);
            turn = -sign*0.2; //turn = -1*sign*0.3; // by Bo
            leftFront =  turn;
            rightFront = - turn;
            leftRear =  turn;
            rightRear = - turn;

            motorHDLeftFront.setPower(leftFront);
            motorHDLeftRear.setPower(leftRear);
            motorHDRightFront.setPower(rightFront);
            motorHDRightRear.setPower(rightRear);

            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            diff = (90-currentAngle);
            //telemetry.addData("current angle", currentAngle);
            //telemetry.addData("diff", diff);
            //telemetry.update();
        }
        motorHDLeftFront.setPower(0);
        motorHDLeftRear.setPower(0);
        motorHDRightFront.setPower(0);
        motorHDRightRear.setPower(0);
    }

    public void turnRight(float power) {
        motorHDLeftFront.setPower(power);
        motorHDLeftRear.setPower(power);

    }
    public void turnLeft(float power) {
        motorHDRightFront.setPower(power);
        motorHDRightRear.setPower(power);
    }
}

