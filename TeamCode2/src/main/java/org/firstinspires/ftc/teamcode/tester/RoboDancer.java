package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
@Config
public class RoboDancer extends OpMode {

    RoboDancerConfig hw;
    SampleMecanumDrive drive;
    SoundManager soundManager;
    ElapsedTime macarenaCD = null;

    double coeff = 0.8;

    public static double dr1 = 0.75;
    public static double dr2 = 0.3;
    public static double dr3 = 0.69;

    public static double st1 = 0.05;
    public static double st2 = 1;
    public static double st3 = 0.68;

    public static double cap = 0.53;

    Thread spin = null;
    @Override
    public void init() {
        hw = new RoboDancerConfig(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        /*soundManager = new SoundManager(hardwareMap);

        telemetry.addData("a ajuns aici", "aici");

        if(!soundManager.addFile("macarena"))
            telemetry.addData("macarena", "Not found(add macarena to src/res/raw)");*/

        hw.St1.setPosition(st1);
        hw.St2.setPosition(st2);
        hw.St3.setPosition(st3);

        hw.Dr1.setPosition(dr1);
        hw.Dr2.setPosition(dr2);
        hw.Dr3.setPosition(dr3);

        hw.cap.setPosition(cap);
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
        double RB = hw.clipPower(drive - strafe - rotate) * coeff;
        double LB = hw.clipPower(drive + strafe + rotate) * coeff;
        double LF = hw.clipPower(drive + strafe + rotate) * coeff;

        hw.leftFront.setPower(LF);
        hw.leftBack.setPower(LB);
        hw.rightFront.setPower(RF);
        hw.rightBack.setPower(RB);

        if(gamepad1.right_bumper){
            hw.cap.setPosition(0.4);
            waveStanga(1);
            hw.cap.setPosition(0.7);
            waveDreapta(1);
        } else if(gamepad1.left_bumper){
            //muie visoiu
            dans(1);
        } else if(gamepad1.x) {
            macarenaDance(1);
        } else {
            hw.Dr1.setPosition(dr1);
            hw.St1.setPosition(st1);
            hw.Dr2.setPosition(dr2);
            hw.St2.setPosition(st2);
            hw.Dr3.setPosition(dr3);
            hw.St3.setPosition(st3);
            hw.cap.setPosition(cap);
        }

        telemetry.addData("dr1", hw.Dr1.getPosition());
        telemetry.addData("dr2", hw.Dr2.getPosition());
        telemetry.addData("dr3", hw.Dr3.getPosition());
        telemetry.addLine();
        telemetry.addData("st1", hw.St1.getPosition());
        telemetry.addData("st2", hw.St2.getPosition());
        telemetry.addData("st3", hw.St3.getPosition());

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

    public void waveDreapta(int nrRep) {
        hw.Dr1.setPosition(0);
        int cont = 0;
        while(cont < nrRep){
            hw.Dr3.setPosition(0.9);
            waitTimer(400);
            hw.Dr3.setPosition(dr3);
            waitTimer(400);
            cont++;
        }
        hw.Dr1.setPosition(dr1);
    }

    public void waveStanga(int nrRep) {
        hw.St1.setPosition(0.8);
        int cont = 0;
        while(cont < nrRep){
            hw.St3.setPosition(0.53);
            waitTimer(400);
            hw.St3.setPosition(st3);
            waitTimer(400);
            cont++;
        }
        hw.St1.setPosition(st1);
    }


    public void dans(int nrRep){
        int cont = 0;
        while(cont < nrRep){
            hw.Dr1.setPosition(0);
            hw.St1.setPosition(0.8);
            waitTimer(400);
            hw.Dr1.setPosition(dr1);
            hw.St1.setPosition(st1);
            waitTimer(400);
            hw.Dr3.setPosition(0.9);
            hw.St3.setPosition(0.53);
            waitTimer(400);
            hw.Dr1.setPosition(0);
            hw.St1.setPosition(0.8);
            waitTimer(400);
            hw.Dr3.setPosition(dr3);
            hw.St3.setPosition(st3);
            cont++;
        }
    }

    public void macarenaDance(int NrReps){

        hw.St1.setPosition(st1);
        hw.St2.setPosition(st2);
        hw.St3.setPosition(st3);

        hw.Dr1.setPosition(dr1);
        hw.Dr2.setPosition(dr2);
        hw.Dr3.setPosition(dr3);

        waitTimer(1000);

        hw.Dr1.setPosition(0);
        hw.St1.setPosition(0.5);
        hw.Dr3.setPosition(1);
        hw.St3.setPosition(0.3);

        waitTimer(1000);

        hw.Dr2.setPosition(0.65);
        hw.St2.setPosition(0.7);

        waitTimer(1000);

        hw.Dr2.setPosition(0);
        hw.St2.setPosition(0);
        hw.Dr3.setPosition(0.3);
        hw.St3.setPosition(1);

        waitTimer(1000);

        hw.Dr2.setPosition(dr2);
        hw.St2.setPosition(st2);
        hw.Dr3.setPosition(1);
        hw.St3.setPosition(0.3);

        waitTimer(1000);

        hw.Dr1.setPosition(0.6);
        hw.St1.setPosition(0.55);

        waitTimer(1000);
    }

    public void waitTimer(int miliseconds){
        ElapsedTime timer;
        timer = new ElapsedTime();
        while(timer.milliseconds() < miliseconds){
            ;
        }
    }
}