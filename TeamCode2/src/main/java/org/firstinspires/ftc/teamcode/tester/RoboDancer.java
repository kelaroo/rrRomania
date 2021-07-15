package org.firstinspires.ftc.teamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class RoboDancer extends OpMode {

    RoboDancerConfig hw;
    SoundManager soundManager;
    ElapsedTime macarenaCD = null;

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
        if(gamepad1.left_stick_button){
            if(macarenaCD == null || macarenaCD.milliseconds() >= 180000){
                soundManager.playSound("macarena");
                macarenaCD = new ElapsedTime();
            }
        }
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