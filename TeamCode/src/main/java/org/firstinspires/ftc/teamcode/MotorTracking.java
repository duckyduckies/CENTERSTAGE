package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTrackingDebug", group = "Iterative Opmode")
public class MotorTracking extends OpMode {
/*    DcMotor motor;
    double ticks = 2789.2;
    double newTarget;*/

    public static Attachments iRobot = new Attachments();


    @Override
    public void init(){
        /*motor = hardwareMap.dcMotor.get("motor1");
        telemetry.addData("Hardware", "Ready to rock!");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        iRobot.initialize(hardwareMap);
        iRobot.resetEncoder();
        iRobot.forwardMode();
    }

    @Override
    public void loop(){
        if(gamepad1.x){  //left
            iRobot.forwardMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(1500, 1500, 0.3); // move forward
            iRobot.resetEncoder();
            iRobot.moveRobot(-550, 550, 0.3); // turn left
            iRobot.resetEncoder();
            iRobot.moveRobot(200, 200, 0.3); // move forwards a bit
            // reverse action
            iRobot.backMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(200, 200, 0.3); // move forwards a bit
            iRobot.resetEncoder();
            iRobot.moveRobot(-550, 550, 0.3); // move forwards a bit
            iRobot.resetEncoder();
            iRobot.moveRobot(1500, 1500, 0.3); // move forwards a bit

            //iRobot.moveRobot(200, -200, 0.3); // move backwards a bit
            //iRobot.moveRobot(-640, 640, 0.3); // turn left
            //encoder(2);

        }

        if(gamepad1.y)  // right
        {
            iRobot.forwardMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(1500, 1500, 0.3); // move forward
            iRobot.resetEncoder();
            iRobot.moveRobot(550, -550, 0.3); // turn right
            iRobot.resetEncoder();
            iRobot.moveRobot(200, 200, 0.3); // move forwards a bit
            //reverse action
            iRobot.backMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(200, 200, 0.3); // move forwards a bit
            iRobot.resetEncoder();
            iRobot.moveRobot(550, -550, 0.3); // move forwards a bit
            iRobot.resetEncoder();
            iRobot.moveRobot(1500, 1500, 0.3); // move forwards a bit

            //go backbase  // right close
            iRobot.forwardMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(550, -550, 0.3); // turn right
            iRobot.resetEncoder();
            iRobot.moveRobot(2000,2000, 0.3); // move forwards a bit
        }

        if(gamepad1.a){ // straight
            iRobot.forwardMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(1800, 1800, 0.3); // move forward
            //reverse action
            iRobot.backMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(1800, 1800, 0.3); // move forwards a bit

        }

        if(gamepad1.b){
            iRobot.resetEncoder();
            iRobot.moveRobot(1800, 1800, 0.3); // move forward
            iRobot.resetEncoder();
            iRobot.moveRobot(-1800, -1800, 0.3); // move forwards a bit

        }
    }

   /* public void encoder(int turnage){
        newTarget = ticks/turnage;
        motor.setTargetPosition((int)newTarget);
        motor.setPower(0.3);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }*/

}
