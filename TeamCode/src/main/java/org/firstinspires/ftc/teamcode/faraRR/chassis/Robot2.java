package org.firstinspires.ftc.teamcode.faraRR.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class Robot2 {
    public DcMotor rightFront; // eh 3
    public DcMotor rightBack; // ch 1
    public DcMotor leftBack; // eh 1
    public DcMotor leftFront; // eh 0
    public List<DcMotor> lDriveMotors;

    public Robot2(HardwareMap hw) {
        rightFront = hw.get(DcMotor.class, "right_front");
        rightBack = hw.get(DcMotor.class, "right_back");
        leftBack = hw.get(DcMotor.class, "left_back");
        leftFront = hw.get(DcMotor.class, "left_front");
        lDriveMotors = Arrays.asList(rightFront, rightBack, leftBack, leftFront);

        for(DcMotor motor: lDriveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
