package org.firstinspires.ftc.teamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tester.HardwareConfigRobotMic;

@TeleOp
public class Scrumiera extends OpMode {
    HardwareConfigRobotMic hw;

    @Override
    public void init() {
        hw=new HardwareConfigRobotMic(hardwareMap);
    }

    @Override
    public void loop() {
        /*//Driver
        double drive=gamepad1.left_stick_y;
        double rotate=gamepad1.right_stick_x;

        double rightFront=drive - rotate;
        double leftFront=-drive - rotate;
        double rightBack=-drive - rotate;
        double leftBack=drive - rotate;

        hw.rightBack.setPower(rightBack);
        hw.leftFront.setPower(leftFront);
        hw.leftBack.setPower(leftBack);
        hw.rightFront.setPower(rightFront);
        telemetry.addData("rightBack" , rightBack);*/

        //Fan
        double invarte=1;
        if(gamepad1.y){
            hw.fan.setPower(invarte);
        }else if(gamepad1.x){
            hw.fan.setPower(0);
            hw.fan.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //Ashtray
       /* if(gamepad1.b){
            hw.ashTray.setPosition(0.5);
        }else if(gamepad1.a){
            hw.ashTray.setPosition(0.3);
        }*/

    }
}
