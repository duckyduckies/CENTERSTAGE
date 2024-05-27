package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpDcMotorTurorial extends OpMode {

    DcMotor motor1;

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor1");

       //motor1.setDirection(DcMotor.Direction.FORWARD);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        float motorPower = 0.6f;

        if(gamepad2.left_stick_y > 0.1)
        {
            motor1.setDirection(DcMotor.Direction.FORWARD);
            motor1.setPower(motorPower);
        }
        else if(gamepad2.right_stick_y > 0.1) {
            motor1.setDirection(DcMotor.Direction.REVERSE);
            motor1.setPower(motorPower);
        }
        else
        {
            motor1.setPower(0);
        }
    }
}
