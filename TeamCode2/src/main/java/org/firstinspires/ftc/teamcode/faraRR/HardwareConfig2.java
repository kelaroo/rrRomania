package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig2.lansatCoeff;

public class HardwareConfig2 {
    public DcMotor rightFront; // eh 3
    public DcMotor rightBack; // ch 1
    public DcMotor leftBack; // eh 1
    public DcMotor leftFront; // eh 0
    public List<DcMotor> lDriveMotors;

    public Servo intake2; // ch 0

    // Expansion Hub
    public DcMotor intake; // eh 2
    public DcMotorEx lansat; // ch 0

    public Servo cuva; // eh 1
    public Servo impins; // ch 1
    //public Servo intake3; // eh 0
    public DcMotor intake3;

    public Servo bratWobble; // eh 2
    public Servo clawWobble; // eh 3

    public Servo bratOprit;
    public Servo baraOprit;

    public HardwareConfig2(HardwareMap hw) {
        /// Control Hub
        rightFront = hw.get(DcMotor.class, "right_front");
        rightBack = hw.get(DcMotor.class, "right_back");
        leftBack = hw.get(DcMotor.class, "left_back");
        leftFront = hw.get(DcMotor.class, "left_front");
        lDriveMotors = Arrays.asList(rightFront, rightBack, leftBack, leftFront);

        for(DcMotor motor: lDriveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        intake2 = hw.get(Servo.class, "intake2");

        /// Expansion Hub
        intake = hw.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lansat = hw.get(DcMotorEx.class, "lansat");
        lansat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lansat.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lansatCoeff);
        //lansat.setDirection(DcMotorSimple.Direction.REVERSE);

        cuva = hw.get(Servo.class, "cuva");
        impins = hw.get(Servo.class, "impins");
        intake3 = hw.get(DcMotor.class, "intake3");
        intake3.setDirection(DcMotorSimple.Direction.REVERSE);

        bratWobble = hw.get(Servo.class, "bratWobble");
        clawWobble = hw.get(Servo.class, "clawWobble");

        bratOprit = hw.get(Servo.class, "bratOprit");
        baraOprit = hw.get(Servo.class, "baraOprit");

        // Servo Init
        //cuva.setPosition(0.5);
    }

    public double clipPower(double power) {
        return (power > 1)? 1: ((power < -1)? -1: power);
    }
}
