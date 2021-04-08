package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.solvers.BracketedRealFieldUnivariateSolver;

import java.util.Arrays;
import java.util.List;

public class HardwareConfig {
    public DcMotor rightFront;
    public DcMotor rightBack;
    public DcMotor leftBack;
    public DcMotor leftFront;
    public List<DcMotor> lDriveMotors;

    public Servo intake2;

    // Expansion Hub
    public DcMotor intake;
    public DcMotor lansat;

    public Servo cuva;
    public Servo impins;
    public Servo intake3;

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

        /// Expansion Hub
        intake = hw.get(DcMotor.class, "odoRight");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lansat = hw.get(DcMotor.class, "odoCenter");
        lansat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cuva = hw.get(Servo.class, "cuva");
        impins = hw.get(Servo.class, "impins");
        intake3 = hw.get(Servo.class, "intake3");

        // Servo Init
        //cuva.setPosition(0.5);
    }

    public double clipPower(double power) {
        return (power > 1)? 1: ((power < -1)? -1: power);
    }
}
