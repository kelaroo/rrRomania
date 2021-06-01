package org.firstinspires.ftc.teamcode.faraRR;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig2.*;
@TeleOp
public class TwoDriver2 extends OpMode {

    HardwareConfig2 hw;
    SampleMecanumDrive drive;

    double coeff = COEFF_SPEED_HIGH;

    enum CuvaState {
        SUS, JOS
    }
    CuvaState cuvaState = CuvaState.SUS;

    enum ShootState {
        SHOOTING, IDLE
    }
    volatile ShootState shootState = ShootState.IDLE;

    enum LansatState {
        IDLE, SHOOTING, POWER_SHOT
    }
    LansatState lansatState = LansatState.IDLE;

    enum RobotState {
        DRIVER, POWERSHOTS, MOVE_TO
    }
    RobotState robotState = RobotState.DRIVER;
    Thread tAutoPS = null;

    boolean manualOverride = false;

    Thread tOneShot = null;

    @Override
    public void init() {
        hw = new HardwareConfig2(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        hw.cuva.setPosition(CUVA_JOS);
        cuvaState = CuvaState.JOS;
    }

    @Override
    public void loop() {

        drive.update();

        if(gamepad1.b && gamepad1.dpad_right)
            manualOverride = true;

        if(robotState == RobotState.DRIVER) {
            if(!manualOverride && gamepad1.dpad_up && gamepad1.y) {
                robotState = RobotState.POWERSHOTS;
                tAutoPS = new Thread(new AutoPowerShots());
                tAutoPS.start();
                return;
            } else if(!manualOverride && gamepad1.dpad_left && gamepad1.b) {
                robotState = RobotState.MOVE_TO;
                Thread tMovePS = new Thread(new MoveToPowerShot());
                tMovePS.start();
                return;
            }
        } else
            return;

        /// Driver 1
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if(gamepad1.b)
            coeff = COEFF_SPEED_HIGH;
        if(gamepad1.x)
            coeff = COEFF_SPEED_LOW;

        double RF = hw.clipPower(drive - strafe - rotate) * coeff;
        double RB = hw.clipPower(drive + strafe - rotate) * coeff;
        double LB = hw.clipPower(drive - strafe + rotate) * coeff;
        double LF = hw.clipPower(drive + strafe + rotate) * coeff;

        hw.leftFront.setPower(LF);
        hw.leftBack.setPower(LB);
        hw.rightFront.setPower(RF);
        hw.rightBack.setPower(RB);

        /*if(gamepad1.right_bumper) {
            hw.bratOprit.setPosition(BRAT_OPRIT_SUS);
            hw.baraOprit.setPosition(BARA_OPRIT_STATIONARY);
        } else if(gamepad1.left_bumper) {
            hw.bratOprit.setPosition(BRAT_OPRIT_JOS);
            hw.baraOprit.setPosition(BARA_OPRIT_SUCK);
        } else if(gamepad1.left_trigger > 0.3) {
            hw.bratOprit.setPosition(BRAT_OPRIT_C);
            hw.baraOprit.setPosition(BARA_OPRIT_SUCK);
        }*/

        /// Driver 2

        // Intake
        if(cuvaState == CuvaState.JOS) {
            if(gamepad2.left_trigger > 0.2) {
                hw.intake.setPower(INTAKE_SUCK);
                hw.intake2.setPosition(INTAKE2_RIGHT);
                hw.intake3.setPower(INTAKE3_SUCK);
            } else if(gamepad2.right_trigger > 0.2) {
                hw.intake.setPower(-INTAKE_SUCK);
                hw.intake2.setPosition(INTAKE2_LEFT);
                hw.intake3.setPower(-INTAKE3_SUCK);
            } else {
                hw.intake.setPower(0);
                hw.intake2.setPosition(INTAKE2_STATIONARY);
                hw.intake3.setPower(0);
            }
        } else { // cuva e sus
            if(gamepad2.left_trigger > 0.2) {
                hw.intake3.setPower(INTAKE3_SUCK);
            } else if(gamepad2.right_trigger > 0.2) {
                hw.intake3.setPower(-INTAKE3_SUCK);
            } else {
                hw.intake.setPower(0);
                hw.intake2.setPosition(INTAKE2_STATIONARY);
                hw.intake3.setPower(0);
            }

            // Matura
            hw.baraOprit.setPosition(BARA_OPRIT_STATIONARY);
        }

        // Cuva
        if(gamepad2.y) {
            hw.cuva.setPosition(CUVA_SUS);
            cuvaState = CuvaState.SUS;
        } else if(gamepad2.a) {
            hw.cuva.setPosition(CUVA_JOS);
            cuvaState = CuvaState.JOS;
        }

        // Impins
        if(gamepad2.left_bumper && cuvaState == CuvaState.SUS && shootState == ShootState.IDLE) {
            /*Thread tAutoShoot = new Thread(new OneButtonShoot());
            tAutoShoot.start();*/
            tOneShot = new Thread(new OneShotOnly());
            tOneShot.start();
        } else if(shootState == ShootState.IDLE && cuvaState == CuvaState.SUS) {
            if(gamepad2.x)
                hw.impins.setPosition(IMPINS_FWD);
            else
                hw.impins.setPosition(IMPINS_BWD);
        } else if(shootState == ShootState.IDLE){
            hw.impins.setPosition(IMPINS_SECOND);
        }

        // Lansat
        // SWTICH VITEZA POWERSHOT BY CEI 2 TRAPPERI
        if(gamepad2.right_bumper)
            lansatState = LansatState.SHOOTING;
        else if(gamepad2.b)
            lansatState = LansatState.POWER_SHOT;
        else
            lansatState = LansatState.IDLE;

        if(lansatState == LansatState.SHOOTING)
            //hw.lansat.setPower(LANSAT_POWER);
            hw.lansat.setVelocity(LANSAT_SPEED);
        else if(lansatState == LansatState.POWER_SHOT)
            hw.lansat.setVelocity(LANSAT_SPEED_PS);
        else
            hw.lansat.setPower(0);

        // Wobble Arm
        if(gamepad2.dpad_up) {
            cuvaState = CuvaState.JOS;
            hw.cuva.setPosition(CUVA_JOS);
            hw.bratWobble.setPosition(BRAT_SUS);
        } else if(gamepad2.dpad_down) {
            hw.bratWobble.setPosition(BRAT_JOS);
        }

        // Wobble claw
        if(gamepad2.dpad_right) {
            hw.clawWobble.setPosition(CLAW_PRINS);
        } else if(gamepad2.dpad_left && !gamepad2.dpad_up) {
            hw.clawWobble.setPosition(CLAW_LASAT);
        }

        // Matura
        if(gamepad1.right_bumper || cuvaState == CuvaState.SUS) {
            hw.bratOprit.setPosition(BRAT_OPRIT_SUS);
            hw.baraOprit.setPosition(BARA_OPRIT_STATIONARY);
        } else if(gamepad1.left_bumper && cuvaState == CuvaState.JOS) {
            hw.bratOprit.setPosition(BRAT_OPRIT_JOS);
            hw.baraOprit.setPosition(BARA_OPRIT_SUCK);
        }
    }

    void waitTimer(int millis) {
        ElapsedTime timer = new ElapsedTime();
        while(timer.milliseconds() < millis)
            ;
    }

    void shoot() {
        ElapsedTime timer = new ElapsedTime();
        hw.impins.setPosition(IMPINS_FWD);
        timer.reset();
        while(timer.milliseconds() < 250)
            ;

        hw.impins.setPosition(IMPINS_BWD);
        timer.reset();
        while(timer.milliseconds() < 250)
            ;
    }

    void shootSlow() {
        ElapsedTime timer = new ElapsedTime();
        hw.impins.setPosition(IMPINS_FWD);
        timer.reset();
        while(timer.milliseconds() < 800)
            ;

        hw.impins.setPosition(IMPINS_BWD);
        timer.reset();
        while(timer.milliseconds() < 250)
            ;
    }

    private class OneButtonShoot implements Runnable {

        @Override
        public void run() {
            telemetry.addData("Thread", "started");
            ElapsedTime timer = new ElapsedTime();
            shootState = ShootState.SHOOTING;

            for(int i = 1; i <= 3; i++) {
                if(cuvaState == CuvaState.JOS)
                    break;

                telemetry.addData("Thread", String.format("Shoot %d", i));
                hw.impins.setPosition(IMPINS_FWD);
                timer.reset();
                while(timer.milliseconds() < 250)
                    ;

                hw.impins.setPosition(IMPINS_BWD);
                timer.reset();
                while(timer.milliseconds() < 250)
                    ;
            }

            shootState = ShootState.IDLE;
            telemetry.update();
        }
    }

    private class OneShotOnly implements Runnable {

        @Override
        public void run() {
            ElapsedTime timer = new ElapsedTime();
            shootState = ShootState.SHOOTING;

            hw.impins.setPosition(IMPINS_FWD);
            while(timer.milliseconds() < 600)
                ;
            hw.impins.setPosition(IMPINS_BWD);

            shootState = ShootState.IDLE;
        }
    }

    private class AutoPowerShots implements Runnable {

        @Override
        public void run() {
            telemetry.addData("AutoPowerShots", "started");

            hw.cuva.setPosition(CUVA_SUS);
            cuvaState = CuvaState.SUS;

            robotState = RobotState.POWERSHOTS;

            hw.lansat.setVelocity(LANSAT_SPEED_PS);
            waitTimer(2500);

            shoot();
            waitTimer(800);

            drive.turn(Math.toRadians(6.0));
            shoot();
            waitTimer(800);

            drive.turn(Math.toRadians(-8.3));
            shoot();

            hw.lansat.setPower(0);
            telemetry.update();
            robotState = RobotState.DRIVER;
        }
    }

    private class MoveToPowerShot implements Runnable {

        @Override
        public void run() {
            robotState = RobotState.MOVE_TO;

            hw.lansat.setVelocity(LANSAT_SPEED_PS);
            hw.intake.setPower(0);
            hw.intake2.setPosition(INTAKE2_STATIONARY);
            hw.intake3.setPower(INTAKE3_SUCK);
            hw.cuva.setPosition(CUVA_SUS);
            waitTimer(1700);

            if(manualOverride) {
                robotState = RobotState.DRIVER;
                return;
            }
            shoot();

            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(9)
                    .addDisplacementMarker(()->{shootSlow();})
                    .build();

            if(manualOverride) {
                robotState = RobotState.DRIVER;
                return;
            }
            drive.followTrajectory(traj);

            traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(9)
                    .addDisplacementMarker(()->{shoot();})
                    .build();

            if(manualOverride) {
                robotState = RobotState.DRIVER;
                return;
            }
            drive.followTrajectory(traj);

            robotState = RobotState.DRIVER;
        }
    }
}
