package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Attachments {
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDriveMotor, rightDriveMotor, liftMotor1, liftMotor2;
    public Servo clawServo, armServo; //, camServo;
    public Rev2mDistanceSensor rightDistance, leftDistance, clawRightDistance, clawLeftDistance, clawDistance;

    public void initialize(HardwareMap hardwareMap) {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Motors
        leftDriveMotor = hardwareMap.dcMotor.get("motor1");
        rightDriveMotor = hardwareMap.dcMotor.get("motor2");
        liftMotor1 = hardwareMap.get(DcMotorEx.class,"motor3");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "motor4");


        // Servos
        clawServo =  hardwareMap.get(Servo.class, "servo1");
        armServo = hardwareMap.get(Servo.class, "servo2");


        // Sensors

        // Motor Behavior
        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    /* --------------------------------------- ACCESSORS --------------------------------------- */
//    public double getRightDistance() {return rightDistance.getDistance(DistanceUnit.INCH);}
//    public double getLeftDistance() {return rightDistance.getDistance(DistanceUnit.INCH);}
//    public double getClawRightDistance() {return clawRightDistance.getDistance(DistanceUnit.INCH)}
//    public double getClawLeftDistance() {return clawLeftDistance.getDistance(DistanceUnit.INCH)}
    public double getClawDistance() {return clawDistance.getDistance(DistanceUnit.INCH);}

    /* ---------------------------------------- SETTERS ---------------------------------------- */
    public void runLiftMotor(double power) {
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }
    public void setLiftMotor(double power, int position) {
        liftMotor1.setPower(power);
        liftMotor1.setTargetPosition(position);
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setPower(power);
        liftMotor2.setTargetPosition(position);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    public void setClawServo (double position) {clawServo.setPosition(position);}
    //    public void setCamServo (double position) {camServo.setPosition(position);}
    public void setArmServo (double position) {armServo.setPosition(position);}

    public int getLiftMotor1Position() {
        return liftMotor1.getCurrentPosition();
    }

    public int getLiftMotor2Position() {
        return liftMotor2.getCurrentPosition();
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }

    public double getArmPosition() {
        return armServo.getPosition();
    }
}

