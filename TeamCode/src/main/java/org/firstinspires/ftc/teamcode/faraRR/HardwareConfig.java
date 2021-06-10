package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.*;

public class HardwareConfig {
    public DcMotor rightFront;
    public DcMotor rightBack;
    public DcMotor leftBack;
    public DcMotor leftFront;
    public List<DcMotor> lDriveMotors;

    public Servo intake2;
    public Servo bratWobble;
    public Servo clawWobble;
    public Servo bratOprit; // ch 5

    // Expansion Hub
    public DcMotor intake;
    public DcMotorEx lansat;

    public Servo cuva;
    public Servo impins;
    public DcMotor intake3;

    public Servo baraOprit; // eh 3

    public Servo baraS;
    public Servo baraD;


    public HardwareConfig(HardwareMap hw) {
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

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        intake2 = hw.get(Servo.class, "intake2");
        bratWobble = hw.get(Servo.class, "bratWobble");
        clawWobble = hw.get(Servo.class, "clawWobble");

        /// Expansion Hub
        intake = hw.get(DcMotor.class, "odoRight");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lansat = hw.get(DcMotorEx.class, "odoCenter");
        lansat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lansat.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lansatCoeff);

        cuva = hw.get(Servo.class, "cuva");
        impins = hw.get(Servo.class, "impins");
        intake3 = hw.get(DcMotor.class, "intake3");

        baraD = hw.get(Servo.class, "baraD");
        baraS = hw.get(Servo.class, "baraS");

        bratOprit = hw.get(Servo.class, "bratOprit");
        baraOprit = hw.get(Servo.class, "baraOprit");

        // Servo Init
        //cuva.setPosition(0.5);
        bratOprit.setPosition(BRAT_OPRIT_INT);
        baraOprit.setPosition(BARA_OPRIT_INT);
    }

    public double clipPower(double power) {
        return (power > 1)? 1: ((power < -1)? -1: power);
    }
}
