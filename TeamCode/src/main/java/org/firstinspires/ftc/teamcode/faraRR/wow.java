package org.firstinspires.ftc.teamcode.faraRR;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class wow extends OpMode {

    int cnt = 1;
    int cnt2 = 1;
    HardwareConfig hw;

    @Override
    public void init() {
        hw = new HardwareConfig(hardwareMap);
        hw.bratWobble.setPosition(0.25);
        hw.clawWobble.setPosition(0.48);
    }

    @Override
    public void loop() {
        hw.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        double d1 = gamepad1.left_stick_y;
        double d2 = gamepad1.left_stick_x;
        double d3 = gamepad1.right_stick_x;

        double rightfront_powr = d1 + d2 - d3;
        double rightback_powr = d1 - d2 - d3;
        double leftfront_powr = d1 - d2 + d3;
        double leftback_powr = d1 + d2 + d3;

        hw.rightFront.setPower(rightfront_powr);
        hw.rightBack.setPower(rightback_powr);
        hw.leftFront.setPower(leftfront_powr);
        hw.leftBack.setPower(leftback_powr);

        if (gamepad1.left_trigger != 0) {
            hw.intake.setPower(0.75);
            hw.intake3.setPower(0.75);
            hw.intake2.setPosition(0);
        } else {
            hw.intake.setPower(0);
            hw.intake3.setPower(0);
            hw.intake2.setPosition(0.5);
        }
        if (gamepad1.dpad_up) {
            hw.bratWobble.setPosition(0.25);
        }
        if (gamepad1.dpad_down) {
            hw.bratWobble.setPosition(0.655);
        }
        if (gamepad1.dpad_left) {
            hw.clawWobble.setPosition(0.48);
        }
        if (gamepad1.dpad_right) {
            hw.clawWobble.setPosition(0.98);
        }
        if (gamepad1.a) {
            cnt = (cnt + 1) % 2;
            if (cnt == 0)
                hw.cuva.setPosition(0.354);
            else
                hw.cuva.setPosition(0.625);
        }
        if (gamepad1.right_trigger!=0) {
            hw.lansat.setPower(0.675);
        } else if (gamepad1.x) {
            hw.lansat.setPower(0.575);
        } else {
            hw.lansat.setPower(0);
        }

    }
}
