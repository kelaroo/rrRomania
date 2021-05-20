package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.*;

@TeleOp
public class TwoDriver extends OpMode {

    HardwareConfig hw;
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
        DRIVER, POWERSHOTS
    }
    RobotState robotState = RobotState.DRIVER;
    Thread tAutoPS = null;

    @Override
    public void init() {
        hw = new HardwareConfig(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
        hw.bratOprit.setPosition(BRAT_OPRIT_EXT);
        hw.baraOprit.setPosition(BARA_OPRIT_EXT);

        hw.cuva.setPosition(CUVA_JOS);
        cuvaState = CuvaState.JOS;
    }

    @Override
    public void loop() {
        drive.update();

        if(robotState == RobotState.DRIVER && gamepad1.y && gamepad1.dpad_up) {
            tAutoPS = new Thread(new AutoPowerShots());
            robotState = RobotState.POWERSHOTS;
            tAutoPS.start();
        }

        if(robotState != RobotState.DRIVER)
            return;

        /// Driver 1
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if(gamepad1.b)
            coeff = COEFF_SPEED_HIGH;
        if(gamepad1.x)
            coeff = COEFF_SPEED_LOW;

        // Brate
        if(gamepad1.left_trigger > 0.2) {
            hw.baraD.setPosition(BARAD_INT);
            hw.baraS.setPosition(BARAS_INT);
        } else if(gamepad1.right_trigger > 0.2) {
            hw.baraD.setPosition(BARAD_EXT);
            hw.baraS.setPosition(BARAS_EXT);
        }

        double RF = hw.clipPower(drive - strafe - rotate) * coeff;
        double RB = hw.clipPower(drive + strafe - rotate) * coeff;
        double LB = hw.clipPower(drive - strafe + rotate) * coeff;
        double LF = hw.clipPower(drive + strafe + rotate) * coeff;

        hw.leftFront.setPower(LF);
        hw.leftBack.setPower(LB);
        hw.rightFront.setPower(RF);
        hw.rightBack.setPower(RB);

        /// Driver 2

        // Intake
        if(cuvaState == CuvaState.JOS) {
            if(gamepad2.left_trigger > 0.2) {
                hw.intake.setPower(INTAKE_SUCK);
                hw.intake2.setPosition(INTAKE2_RIGHT);
                hw.intake3.setPosition(INTAKE3_RIGHT);
            } else if(gamepad2.right_trigger > 0.2) {
                hw.intake.setPower(-INTAKE_SUCK);
                hw.intake2.setPosition(INTAKE2_LEFT);
                hw.intake3.setPosition(INTAKE3_LEFT);
            } else {
                hw.intake.setPower(0);
                hw.intake2.setPosition(INTAKE2_STATIONARY);
                hw.intake3.setPosition(INTAKE3_STATIONARY);
            }
        } else {
            if(gamepad2.left_trigger > 0.2) {
                hw.intake3.setPosition(INTAKE3_RIGHT);
            } else if(gamepad2.right_trigger > 0.2) {
                hw.intake3.setPosition(INTAKE3_LEFT);
            } else {
                hw.intake.setPower(0);
                hw.intake2.setPosition(INTAKE2_STATIONARY);
                hw.intake3.setPosition(INTAKE3_STATIONARY);
            }
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
            Thread tAutoShoot = new Thread(new OneButtonShoot());
            tAutoShoot.start();
        } else if(shootState == ShootState.IDLE && cuvaState == CuvaState.SUS) {
            if(gamepad2.x)
                hw.impins.setPosition(IMPINS_FWD);
            else
                hw.impins.setPosition(IMPINS_BWD);
        } else if(shootState == ShootState.IDLE){
            hw.impins.setPosition(IMPINS_SECOND);
        }

        /*// Lansat
        if(gamepad2.right_bumper) {
            hw.lansat.setPower(LANSAT_POWER);
            lansatState = LansatState.SHOOTING;
        } else {
            hw.lansat.setPower(0);
            lansatState = LansatState.IDLE;
        }*/

        // Lansat
        // SWTICH VITEZA POWERSHOT BY CEI 2 TRAPPERI
        if(gamepad2.right_bumper)
            lansatState = LansatState.SHOOTING;
        else if(gamepad2.b)
            lansatState = LansatState.POWER_SHOT;
        else
            lansatState = LansatState.IDLE;

        if(lansatState == LansatState.SHOOTING)
            hw.lansat.setPower(LANSAT_POWER);
        else if(lansatState == LansatState.POWER_SHOT)
            hw.lansat.setPower(LANSAT_POWER_PS);
        else
            hw.lansat.setPower(0);

        // Wobble Arm
        if(gamepad2.dpad_up) {
            hw.bratWobble.setPosition(BRAT_SUS);
        } else if(gamepad2.dpad_down) {
            hw.bratWobble.setPosition(BRAT_JOS);
        }

        // Wobble claw
        if(gamepad2.dpad_right) {
            hw.clawWobble.setPosition(CLAW_PRINS);
        } else if(gamepad2.dpad_left) {
            hw.clawWobble.setPosition(CLAW_LASAT);
        }

        // Bara oprit
        if(gamepad2.left_stick_button) {
            hw.baraOprit.setPosition(BARA_OPRIT_EXT);
            hw.bratOprit.setPosition(BRAT_OPRIT_EXT);
        } else if(gamepad2.right_stick_button) {
            hw.baraOprit.setPosition(BARA_OPRIT_INT);
            hw.bratOprit.setPosition(BRAT_OPRIT_INT);
        }
    }

    private class OneButtonShoot implements Runnable {

        @Override
        public void run() {
            telemetry.addData("Thread", "started");
            ElapsedTime timer = new ElapsedTime();
            shootState = ShootState.SHOOTING;

            for(int i = 1; i <= 3 && cuvaState == CuvaState.SUS; i++) {
                telemetry.addData("Thread", String.format("Shoot %d", i));
                hw.impins.setPosition(IMPINS_FWD);
                timer.reset();
                while(timer.milliseconds() < 250)
                    continue;

                hw.impins.setPosition(IMPINS_BWD);
                timer.reset();
                while(timer.milliseconds() < 250)
                    continue;
            }

            shootState = ShootState.IDLE;
            telemetry.update();
        }
    }

    private class AutoPowerShots implements Runnable {

        @Override
        public void run() {
            telemetry.addData("AutoPowerShots", "started");

            hw.cuva.setPosition(CUVA_SUS);
            cuvaState = CuvaState.SUS;

            robotState = RobotState.POWERSHOTS;

            // Porneste motor
            hw.lansat.setPower(LANSAT_POWER_PS);
            waitTimer(2500);

            shoot();
            waitTimer(800);

            drive.turn(Math.toRadians(6.0));
            shoot();
            hw.lansat.setPower(LANSAT_POWER_PS-0.03);
            waitTimer(800);

            drive.turn(Math.toRadians(-7.8));
            shoot();

            hw.lansat.setPower(0);
            telemetry.update();
            robotState = RobotState.DRIVER;
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
    }
}