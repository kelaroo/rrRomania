package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_BWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_FWD;

@Config
@Autonomous
public class LansatTuning extends LinearOpMode {
    final static double MAX_RPM = 6000;
    final static double TICKS_PER_REV = 28;

    public static PIDFCoefficients pidCoefficients = new PIDFCoefficients(25, 0.0, 10, 13.45);//MAX_RPM * 60 / TICKS_PER_REV);
    public static double targetVelocity = 1350;
    public static double currentVelocity = 0;


    DcMotorEx lansat;
    Servo impins;

    boolean shooting = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        lansat = hardwareMap.get(DcMotorEx.class, "odoCenter");
        lansat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        impins = hardwareMap.get(Servo.class, "impins");

        lansat.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);

        waitForStart();

        PIDFCoefficients prevCoeff = new PIDFCoefficients(pidCoefficients);
        while(opModeIsActive()) {
            currentVelocity = lansat.getVelocity();

            if(prevCoeff.p != pidCoefficients.p || prevCoeff.i != pidCoefficients.i
                    || prevCoeff.d != pidCoefficients.d || prevCoeff.f != pidCoefficients.f)
                lansat.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);

            if(gamepad1.left_bumper && !shooting) {
                Thread shoot = new Thread(new OneButtonShoot());
                shoot.start();
            }

            if(gamepad1.x)
                impins.setPosition(IMPINS_FWD);
            else if(!shooting)
                impins.setPosition(IMPINS_BWD);

            if(gamepad1.dpad_left)
                lansat.setPower(0);
            if(gamepad1.dpad_right)
                lansat.setVelocity(targetVelocity);

            prevCoeff = new PIDFCoefficients(pidCoefficients);

            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.addData("currVelocity", currentVelocity);

            telemetry.addData("PIDF", lansat.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();
        }
    }

    private class OneButtonShoot implements Runnable {

        @Override
        public void run() {
            ElapsedTime timer = new ElapsedTime();
            shooting = true;

            for(int i = 0; i < 3; i++) {
                impins.setPosition(IMPINS_FWD);
                timer.reset();
                while(timer.milliseconds() < 250)
                    continue;

                impins.setPosition(IMPINS_BWD);
                timer.reset();
                while(timer.milliseconds() < 250)
                    continue;
            }

            shooting = false;
        }
    }
}
