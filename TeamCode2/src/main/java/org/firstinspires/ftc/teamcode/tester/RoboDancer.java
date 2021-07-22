package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
@Config
public class RoboDancer extends OpMode {

    RoboDancerConfig hw;
    SoundManager soundManager;
    ElapsedTime macarenaCD = null;

    double coeff = 0.8;

    public static double dr1 = 0;
    public static double dr2 = 0;
    public static double dr3 = 0;

    public static double st1 = 0;
    public static double st2 = 0;
    public static double st3 = 0;


    @Override
    public void init() {
        hw = new RoboDancerConfig(hardwareMap);

        soundManager = new SoundManager(hardwareMap);

        telemetry.addData("a ajuns aici", "aici");

        if(!soundManager.addFile("macarena"))
            telemetry.addData("macarena", "Not found(add macarena to src/res/raw)");
    }


    /*@Override
    public void start() {
        super.start();
    }*/
    @Override
    public void loop() {

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        double RF = hw.clipPower(drive - strafe - rotate) * coeff;
        double RB = hw.clipPower(drive + strafe - rotate) * coeff;
        double LB = hw.clipPower(drive - strafe + rotate) * coeff;
        double LF = hw.clipPower(drive + strafe + rotate) * coeff;

        hw.leftFront.setPower(LF);
        hw.leftBack.setPower(LB);
        hw.rightFront.setPower(RF);
        hw.rightBack.setPower(RB);

        if(gamepad1.dpad_up) hw.St1.setPosition(st1);
        telemetry.addData(String.valueOf(st1), hw.St1.getPosition());
        if(gamepad1.dpad_right) hw.St2.setPosition(st2);
        telemetry.addData(String.valueOf(st2), hw.St2.getPosition());
        if(gamepad1.dpad_down) hw.St3.setPosition(st3);
        telemetry.addData(String.valueOf(st3), hw.St3.getPosition());

        hw.St1.setPosition(st1);
        hw.St2.setPosition(st2);
        hw.St3.setPosition(st3);

        if(gamepad1.y) hw.Dr1.setPosition(dr1);
        telemetry.addData(String.valueOf(dr1), hw.Dr1.getPosition());
        if(gamepad1.b) hw.Dr2.setPosition(dr2);
        telemetry.addData(String.valueOf(dr2), hw.Dr2.getPosition());
        if(gamepad1.a) hw.Dr3.setPosition(dr3);
        telemetry.addData(String.valueOf(dr3), hw.Dr3.getPosition());

        hw.Dr1.setPosition(dr1);
        hw.Dr2.setPosition(dr2);
        hw.Dr3.setPosition(dr3);

        /*if(gamepad1.left_stick_button){
            if(macarenaCD == null || macarenaCD.milliseconds() >= 180000){
                soundManager.playSound("macarena");
                macarenaCD = new ElapsedTime();
            }
        }*/
        // Wobble claw
       /* if(gamepad2.dpad_right) {
            hw.clawWobble.setPosition(CLAW_PRINS);
        } else if(gamepad2.dpad_left && !gamepad2.dpad_up) {
            hw.clawWobble.setPosition(CLAW_LASAT);
            if(oosCD == null || oosCD.milliseconds() >= 700) {
                soundManager.playSound("oos");
                oosCD = new ElapsedTime();
            }
        }*/
    }
}