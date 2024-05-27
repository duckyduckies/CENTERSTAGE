package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest")
public class ServoTest extends OpMode {

    Servo servo1;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
    }

    @Override
    public void loop() {
        if (gamepad2.y)
        {
            servo1.setPosition(0);
        }
        if (gamepad2.x)
        {
            servo1.setPosition(1);
        }

    }
}
