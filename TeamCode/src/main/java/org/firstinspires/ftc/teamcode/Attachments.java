package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.teamTeleOpCode.wristPosition;
import static org.firstinspires.ftc.teamcode.teamTeleOpCode.clawPosition;
import static org.firstinspires.ftc.teamcode.teamTeleOpCode.iRobot;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Attachments {
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDriveMotor, rightDriveMotor, leftArmMotor, rightArmMotor;

    public Servo clawServo;
    public static Servo  wristServo; //, camServo;

    public Servo droneServo;

    ColorSensor color;

    public Rev2mDistanceSensor rightDistance, leftDistance, clawRightDistance, clawLeftDistance, clawDistance;

    public void initialize(HardwareMap hardwareMap) {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Motors operating the ARM
        leftDriveMotor = hardwareMap.dcMotor.get("motor1");
        rightDriveMotor = hardwareMap.dcMotor.get("motor2");

        // Motors operating the ARM
        leftArmMotor = hardwareMap.get(DcMotorEx.class,"motor3");
        rightArmMotor = hardwareMap.get(DcMotorEx.class, "motor4");

        // Servos operating wrist and claw
      /*  clawServo =  hardwareMap.get(Servo.class, "servo1");
        wristServo = hardwareMap.get(Servo.class, "servo2");
        droneServo = hardwareMap.get(Servo.class, "servo3");*/
        // Sensors
        // Todo: need to add

        // Motor Behavior
        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArmMotor.setDirection(DcMotor.Direction.FORWARD);

        rightArmMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    /* --------------------------------------- ACCESSORS --------------------------------------- */
//    public double getRightDistance() {return rightDistance.getDistance(DistanceUnit.INCH);}
//    public double getLeftDistance() {return rightDistance.getDistance(DistanceUnit.INCH);}
//    public double getClawRightDistance() {return clawRightDistance.getDistance(DistanceUnit.INCH)}
//    public double getClawLeftDistance() {return clawLeftDistance.getDistance(DistanceUnit.INCH)}

    public double getClawDistance() {
        return clawDistance.getDistance(DistanceUnit.INCH);
    }

    /* ---------------------------------------- SETTERS ---------------------------------------- */
    public void runArmMotors(double power) {
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }

    public void setArmMotors(double power, int position) {
        leftArmMotor.setPower(power);
        leftArmMotor.setTargetPosition(position);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setPower(power);
        rightArmMotor.setTargetPosition(position);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void wristDown() {
        wristPosition = Constants.wristDown;
        wristServo.setPosition(wristPosition);
        clawPosition = Constants.clawOpen;
        clawServo.setPosition(clawPosition);
        //setArmMotors(0.3, 0);
    }
    public void pickUpPixel() {
        clawPosition = Constants.clawOpen;
        clawServo.setPosition(clawPosition);
        sleep(100);
        wristPosition = Constants.wristDown;
        wristServo.setPosition(wristPosition);
        sleep(1000);
        clawPosition = Constants.clawClose;
        clawServo.setPosition(clawPosition);
        sleep(1000);
        wristPosition = Constants.wristUp;
        wristServo.setPosition(wristPosition);
    }

    public void setClawServo (double position) {
        clawServo.setPosition(position);
    }

    public void setDroneServo (double position) {droneServo.setPosition(position);}

    public void release() {

        wristPosition = Constants.wristDown;
        wristServo.setPosition(wristPosition);
        setArmMotors(0.7, 0);
        sleep(1000);
        clawPosition = Constants.clawOpen;
        clawServo.setPosition(clawPosition);
        sleep(500);
        setArmMotors(0.7, 1);
    }
    public void rotateClockwise() {
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDriveMotor.setPower(0.5);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor.setPower(0.5);
    }

    public void rotateCounterClockwise() {
        leftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDriveMotor.setPower(0.5);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveMotor.setPower(0.5);
    }



    //    public void setCamServo (double position) {camServo.setPosition(position);}

    public void setWristServo(double position) {
        wristServo.setPosition(position);
    }

    public int getLeftArmPosition() {
        return leftArmMotor.getCurrentPosition();
    }

    public int getRightArmPosition() {
        return rightArmMotor.getCurrentPosition();
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }

    public double getWristPosition() {
        return wristServo.getPosition();
    }

    public void moveRobot(int leftTarget, int rightTarget, double speed){

        leftDriveMotor.setTargetPosition(leftTarget);
        rightDriveMotor.setTargetPosition(rightTarget);

        leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveMotor.setMode((DcMotor.RunMode.RUN_TO_POSITION));

        leftDriveMotor.setPower(speed);
        rightDriveMotor.setPower(speed);

        //sleep(1000);
        while(leftDriveMotor.isBusy()){
            sleep(10);
        }
    }


    public void resetEncoder(){
        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       // rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void forwardMode(){
        leftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void backMode(){
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


}

