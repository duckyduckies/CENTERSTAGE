package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "practice")
public class practice extends OpMode {
    DcMotor motor1;
    Servo servo1;
    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class,"motor1");
        servo1 = hardwareMap.get(Servo.class, "servo1");
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_y > 0.1){
            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
            motor1.setPower(0.3);

        }
        else if(gamepad1.right_stick_y > 0.1){
            //servo1.setPosition(1);
            motor1.setDirection(DcMotorSimple.Direction.FORWARD);
            motor1.setPower(0.3);
        }
        else{
            motor1.setPower(0);
            //servo1.setPosition(0);
        }

    }
}
