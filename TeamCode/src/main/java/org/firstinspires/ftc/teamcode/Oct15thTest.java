package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Oct15thTest extends OpMode {
    Servo servo1;
    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
    }

    @Override
    public void loop() {
        if(gamepad1.cross){
            servo1.setPosition(1);
        }
        else if(gamepad1.a) {
            servo1.setPosition(0.5);
        }
    }
}
