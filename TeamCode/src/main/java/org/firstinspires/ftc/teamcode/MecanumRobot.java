package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.teamTeleOpCode.clawPosition;
import static org.firstinspires.ftc.teamcode.teamTeleOpCode.wristPosition;

import android.util.Size;

//import com.google.blocks.ftcrobotcontroller.runtime.AprilTagAccess;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public class MecanumRobot {
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;
    public ColorSensor colorSensor, colorSensor2;
    public VisionPortal visionPortal;
    public AprilTagProcessor tagProcessor;

    public IMU imu;
    private Telemetry telemetry;
    public int default_red;
    public int default_blue;

    public DcMotor motor1ex;
    public DcMotor motor2ex;
    public DcMotor motor3ex;
    public Servo servo1;
    public Servo servo2;

    public Servo servo3;
    public Servo servo4;

    public void initialize(HardwareMap hardwareMap, Telemetry _telemetry)
    {
        telemetry = _telemetry;
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor1ex = hardwareMap.dcMotor.get("motor1ex");
        motor2ex = hardwareMap.dcMotor.get("motor2ex");
        motor3ex = hardwareMap.dcMotor.get("motor3ex");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");

        colorSensor = hardwareMap.get(ColorSensor.class, "sensorColorRangeR");
        colorSensor.enableLed(true);
        colorSensor2 = hardwareMap.get(ColorSensor.class, "sensorColor2");
        colorSensor2.enableLed(true);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1ex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2ex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3ex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        motor4.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1ex.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2ex.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3ex.setDirection(DcMotorSimple.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        default_red = colorSensor.red();
        default_blue = colorSensor.blue();
        int myAprilTaIdCode = -1;
        int targetAprilTag = 2;
        boolean aprilTagRunning = false;
        // mode 0 : scanning
        // mode 1 : approaching
        int aprilTagMode = 0;
        double desiredDistance = 7;
        boolean aprilTagDetected = false;



        double distance = Double.MAX_VALUE;

    }
    public void intializeAprilTag(HardwareMap hardwareMap)
    {
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
        motor1.setPower(leftFront * powerScale);
        motor2.setPower(leftRear * powerScale);
        motor3.setPower(rightFront * powerScale);
        motor4.setPower(rightRear * powerScale);

        telemetry.addData("Motor 1 Left Front",leftFront * powerScale);
        telemetry.addData("Motor 2 Left Rear", leftRear * powerScale);
        telemetry.addData("Motor 3 Right Front",rightFront * powerScale);
        telemetry.addData("Motor 4 Right Rear", rightRear * powerScale);
    }


    public void pickUpPixel() {
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

    public void releasePixel() {
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

    public int getLeftArmPosition() {
        return motor1ex.getCurrentPosition();
    }

    public int getWristPosition() {
        return motor3ex.getCurrentPosition();
    }


    public void RotateP90() {
        imu.resetYaw(); // set to 0 degree
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentAngle = currentAngle *  180 / Math.PI;
        double leftFront, rightFront, leftRear, rightRear, turn;
        double diff =0;
        diff = (90+currentAngle);
        double sign;
        while (Math.abs(diff)>2){
            sign = diff/Math.abs(diff);
            turn = sign*0.3;
            leftFront =  turn;
            rightFront = -turn;
            leftRear =  turn;
            rightRear =  - turn;

            motor1.setPower(leftFront);
            motor2.setPower(leftRear);
            motor3.setPower(rightFront);
            motor4.setPower(rightRear);

            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            diff = (90+currentAngle);
            //telemetry.addData("current angle", currentAngle);
            //telemetry.addData("diff", diff);
            //telemetry.update();
        }
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }

    public void RotateM90() {
        imu.resetYaw(); // set to 0 degree
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentAngle = currentAngle *  180 / Math.PI;
        double leftFront, rightFront, leftRear, rightRear, turn;
        double diff =0;
        diff = (90-currentAngle);
        double sign;
        while (Math.abs(diff)>2){
            sign = diff/Math.abs(diff);
            turn = -1*sign*0.3;
            leftFront =  turn;
            rightFront = - turn;
            leftRear =  turn;
            rightRear = - turn;

            motor1.setPower(leftFront);
            motor2.setPower(leftRear);
            motor3.setPower(rightFront);
            motor4.setPower(rightRear);

            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            diff = (90-currentAngle);
            //telemetry.addData("current angle", currentAngle);
            //telemetry.addData("diff", diff);
            //telemetry.update();
        }
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
}

