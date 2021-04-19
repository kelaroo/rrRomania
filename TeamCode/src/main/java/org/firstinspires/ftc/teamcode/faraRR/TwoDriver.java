package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.*;

@TeleOp
public class TwoDriver extends OpMode {

    HardwareConfig hw;

    double coeff = COEFF_SPEED_HIGH;

    enum CuvaState {
        SUS, JOS
    }
    CuvaState cuvaState = CuvaState.SUS;

    @Override
    public void init() {
        hw = new HardwareConfig(hardwareMap);
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
/*
        if(gamepad2.left_trigger > 0.2 && cuvaState == CuvaState.JOS) {
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
        }*/

        // Cuva
        if(gamepad2.y) {
            hw.cuva.setPosition(CUVA_SUS);
            cuvaState = CuvaState.SUS;
        } else if(gamepad2.a) {
            hw.cuva.setPosition(CUVA_JOS);
            hw.impins.setPosition(IMPINS_SECOND);
            cuvaState = CuvaState.JOS;
        }

        // Impins
        if(gamepad2.left_bumper && cuvaState == CuvaState.SUS)
            hw.impins.setPosition(IMPINS_FWD);
        else if(cuvaState == CuvaState.SUS)
            hw.impins.setPosition(IMPINS_BWD);

        if(gamepad2.right_bumper) {
            hw.lansat.setPower(LANSAT_POWER);
        } else {
            hw.lansat.setPower(0);
        }

        // Brate
        if(gamepad2.b) {
            hw.baraD.setPosition(BARAD_INT);
            hw.baraS.setPosition(BARAS_INT);
        } else if(gamepad2.x) {
            hw.baraD.setPosition(BARAD_EXT);
            hw.baraS.setPosition(BARAS_EXT);
        }

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
    }
}
