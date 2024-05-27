package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="MotorTracking")
public class MotorTracking extends OpMode {
    DcMotor motor;
    double ticks = 2789.2;
    double newTarget;

    @Override
    public void init(){
        motor = hardwareMap.dcMotor.get("motor1");
        telemetry.addData("Hardware", "Ready to rock!");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){
        if(gamepad1.a){
            encoder(2);
        }
    }

    public void encoder(int turnage){
        newTarget = ticks/turnage;
        motor.setTargetPosition((int)newTarget);
        motor.setPower(0.3);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
