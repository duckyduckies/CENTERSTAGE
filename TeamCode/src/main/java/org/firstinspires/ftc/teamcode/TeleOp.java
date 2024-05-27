package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TeleOp extends OpMode {
    private DcMotor Front = null;
    private DcMotor Back = null;

    @Override
    public void init() {
        Front = hardwareMap.get(DcMotor.class,"Front");
        Back = hardwareMap.get(DcMotor.class,"Back");

        Back.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        Front.setPower(gamepad1.left_stick_y);
        Back.setPower(gamepad1.left_stick_x);
    }
}
