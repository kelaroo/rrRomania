package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig2.*;

@TeleOp
public class TwoDriver2 extends OpMode {

    HardwareConfig2 hw;

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

    @Override
    public void init() {
        hw = new HardwareConfig2(hardwareMap);
    }

    @Override
    public void loop() {

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

        /// Driver 2
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
            hw.intake3.setPower(0   );
        }

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
        } else if(shootState == ShootState.IDLE) {
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
            hw.lansat.setPower(LANSAT_POWER);
        else if(lansatState == LansatState.POWER_SHOT)
            hw.lansat.setPower(LANSAT_POWER_PS);
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
        } else if(gamepad2.dpad_left) {
            hw.clawWobble.setPosition(CLAW_LASAT);
        }
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
}
