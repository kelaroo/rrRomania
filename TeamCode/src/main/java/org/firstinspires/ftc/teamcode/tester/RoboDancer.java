package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonome.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.faraRR.HardwareConfig;
import org.firstinspires.ftc.teamcode.faraRR.TwoDriver;
import org.firstinspires.ftc.teamcode.tester.SoundManager;


@Disabled
@TeleOp
public class RoboDancer extends OpMode {

    HardwareConfig hw;
    SoundManager soundManager;
    ElapsedTime macarenaCD = null;

    @Override
    public void init() {
        hw = new HardwareConfig(hardwareMap);

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