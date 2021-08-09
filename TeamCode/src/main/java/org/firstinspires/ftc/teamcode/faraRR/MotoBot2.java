package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@TeleOp
public class MotoBot2 extends OpMode {
    HardwareConfigMoto hw;
    @Override
    public void init() {
        hw=new HardwareConfigMoto(hardwareMap);
    }

    @Override
    public void loop() {
        //Drive
        double drive=-gamepad1.left_stick_y;
        double strafe=gamepad1.right_stick_x;
        double rotate=gamepad1.left_stick_x;

        double rightFront=drive-strafe-rotate;
        double rightBack=drive-strafe+rotate;
        double leftFront=-drive-strafe-rotate;
        double leftBack=-drive-strafe+rotate;

        hw.leftBack.setPower(leftBack);
        hw.leftFront.setPower(leftFront);
        hw.rightBack.setPower(rightBack);
        hw.rightFront.setPower(rightFront);

        //Cuva
        if(gamepad2.left_bumper){
            hw.cuva.setPosition(0.354);
        }else if(gamepad2.left_trigger>0){
            hw.cuva.setPosition(0.625);
        }
        if (gamepad2.right_trigger>0) {
            hw.launcher.setPower(0.625);
        }else{
            hw.launcher.setPower(0);
        }
        if(gamepad2.right_bumper){
            hw.impins.setPosition(0);
        }else{
            hw.impins.setPosition(0.32);
        }
        //Intake
        if(gamepad2.y){
            hw.intake1.setPower(0.584);
            hw.intake2.setPower(0.584);
            hw.intake.setPosition(1);
        }else if(gamepad2.a){
            hw.intake1.setPower(-0.584);
            hw.intake2.setPower(-0.584);
            hw.intake.setPosition(0);
        }else{
            hw.intake1.setPower(0);
            hw.intake2.setPower(0);
            hw.intake.setPosition(0.5);
        }
        //Wobble
        if(gamepad2.dpad_up){
            hw.bratWobble.setPosition(0.25);
        }else if(gamepad2.dpad_down){
            hw.bratWobble.setPosition(0.655);
        }
        if (gamepad2.dpad_left) {
            hw.clawWobble.setPosition(0.48);
        }else if(gamepad2.dpad_right){
            hw.clawWobble.setPosition(0.98);
        }
    }
}
