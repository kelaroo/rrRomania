package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class Robo8 extends OpMode {

    Servo capac;
    DcMotor rightFront, rightBack, leftFront, leftBack;

    public static double copac = 0.5;

    @Override
    public void init() {
        capac = hardwareMap.get(Servo.class, "capac");

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        /*rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);*/
    }

    @Override
    public void loop() {
        double drive=gamepad1.left_stick_y;
        double rotate=-gamepad1.right_stick_x;

        double RF=drive + rotate;
        double LF=-drive + rotate;
        double RB=drive + rotate;
        double LB=-drive + rotate;

        rightBack.setPower(RB);
        leftFront.setPower(LF);
        leftBack.setPower(LB);
        rightFront.setPower(RF);

        if(gamepad1.dpad_up){
            capac.setPosition(copac);
        } else if(gamepad1.dpad_down) {
            capac.setPosition(0.93);
        }
    }
}
