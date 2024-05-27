package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="MotorTiming")
public class LinearOpTest extends LinearOpMode {
    DcMotor motor1;


    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.dcMotor.get("motor1");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        motor1.setPower(0.8);
        sleep(30000);
        /*motor1.setPower(0);
        sleep(3000);
        motor1.setPower(0.5);
        sleep(5000);
        motor1.setPower(0);*/
    }
}
