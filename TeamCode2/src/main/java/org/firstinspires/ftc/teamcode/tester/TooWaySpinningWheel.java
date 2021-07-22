package org.firstinspires.ftc.teamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Random;

@TeleOp
public class TooWaySpinningWheel extends LinearOpMode {

    DigitalChannel digitalTouch;
    DcMotor base;
    DcMotor wheel;

    @Override
    public void runOpMode() {

        digitalTouch = hardwareMap.get(DigitalChannel.class, "button");

        base = hardwareMap.get(DcMotor.class, "base");
        wheel =hardwareMap.get(DcMotor.class, "wheel");

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {
            if (digitalTouch.getState() == false) {
                ElapsedTime timer = new ElapsedTime();
                while(timer.milliseconds() < getRandIntTimer()){
                    wheel.setPower(0.5);
                }
                wheel.setPower(0);
                waitTimer(200);
            }
        }
    }

    void waitTimer(int millis) {
        ElapsedTime timer = new ElapsedTime();
        while(timer.milliseconds() < millis)
            ;
    }
    int getRandIntTimer(){
        Random random;
        random = new Random();
        return (random.nextInt(10) + 10)*1000;
    }
}