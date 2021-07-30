package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MotoBot extends OpMode {
    HardwareConfigMoto cfg;

    @Override
    public void init() {
        cfg=new HardwareConfigMoto(hardwareMap);
    }

    @Override
    public void loop() {
        //Drive
        double drive=-gamepad1.left_stick_y;
        double strafe=gamepad1.left_stick_x;
        double rotate=gamepad1.right_stick_x;

        double rightFront=drive-rotate-strafe;
        double leftFront=-drive-rotate-strafe;
        double rightBack=drive-rotate+strafe;
        double leftBack=-drive-rotate+strafe;

        cfg.rightFront.setPower(rightFront);
        cfg.rightBack.setPower(rightBack);
        cfg.leftFront.setPower(leftFront);
        cfg.leftBack.setPower(leftBack);
        //Intake
        if(gamepad1.right_bumper){
            cfg.intake1.setPower(0.584);
            cfg.intake2.setPower(0.584);
            cfg.intake.setPosition(1);
        }else if(gamepad1.right_trigger>0){
            cfg.intake1.setPower(-0.584);
            cfg.intake2.setPower(-0.584);
            cfg.intake.setPosition(0);
        }else{
            cfg.intake1.setPower(0);
            cfg.intake2.setPower(0);
            cfg.intake.setPosition(0.5);
        }
        //Launcher
        if(gamepad1.x){
            cfg.cuva.setPosition(0.354);
        }else if(gamepad1.a){
            cfg.cuva.setPosition(0.625);
        }
        if(gamepad1.y){
            cfg.impins.setPosition(0);
        }else{
            cfg.impins.setPosition(0.32);
        }
        if(gamepad1.b) {
            cfg.launcher.setPower(0.675);
        }else{
            cfg.launcher.setPower(0);
        }
        //Wobble
        if(gamepad1.dpad_up){
            cfg.bratWobble.setPosition(0.25);
        }else if(gamepad1.dpad_down){
            cfg.bratWobble.setPosition(0.655);
        }
        if(gamepad1.dpad_left){
            cfg.clawWobble.setPosition(0.48);
        }else if(gamepad1.dpad_right){
            cfg.clawWobble.setPosition(0.98);
        }
    }
}
