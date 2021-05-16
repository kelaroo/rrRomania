package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig2.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.faraRR.chassis.Robot2;

@TeleOp
public class ConifgTest2 extends OpMode {

    Robot2 hw;

    @Override
    public void init() {
        hw = new Robot2(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_right)
            hw.rightFront.setPower(COEFF_SPEED_LOW);
        else
            hw.rightFront.setPower(0);

        if(gamepad1.dpad_up)
            hw.rightBack.setPower(COEFF_SPEED_LOW);
        else
            hw.rightBack.setPower(0);

        if(gamepad1.dpad_left)
            hw.leftBack.setPower(COEFF_SPEED_LOW);
        else
            hw.leftBack.setPower(0);

        if(gamepad1.dpad_down)
            hw.leftFront.setPower(COEFF_SPEED_LOW);
        else
            hw.leftFront.setPower(0);

        telemetry.addData("rightFront", hw.rightFront.getCurrentPosition());
        telemetry.addData("rightBack", hw.rightBack.getCurrentPosition());
        telemetry.addData("leftFront", hw.leftFront.getCurrentPosition());
        telemetry.addData("leftBack", hw.leftBack.getCurrentPosition());
    }
}
