package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ConfigTest extends OpMode {

    HardwareConfig hw;

    DcMotor odoRight;
    DcMotor odoCenter;
    DcMotor odoLeft;

    @Override
    public void init() {
        hw = new HardwareConfig(hardwareMap);

        /*odoRight = hardwareMap.get(DcMotor.class, "odoRight");
        odoCenter = hardwareMap.get(DcMotor.class, "odoCenter");
        odoLeft = hardwareMap.get(DcMotor.class, "odoLeft");

        odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_right)
            hw.rightFront.setPower(0.2);
        else
            hw.rightFront.setPower(0);

        if(gamepad1.dpad_up)
            hw.rightBack.setPower(0.2);
        else
            hw.rightBack.setPower(0);

        if(gamepad1.dpad_left)
            hw.leftBack.setPower(0.2);
        else
            hw.leftBack.setPower(0);

        if(gamepad1.dpad_down)
            hw.leftFront.setPower(0.2);
        else
            hw.leftFront.setPower(0);

        telemetry.addData("rightFront", hw.rightFront.getCurrentPosition());
        telemetry.addData("rightBack", hw.rightBack.getCurrentPosition());
        telemetry.addData("leftBack", hw.leftBack.getCurrentPosition());
        telemetry.addData("leftFront", hw.leftFront.getCurrentPosition());

        telemetry.addData("odoRight", odoRight.getCurrentPosition());
        telemetry.addData("odoCenter", odoCenter.getCurrentPosition());
        telemetry.addData("odoLeft", odoLeft.getCurrentPosition());
    }

}
