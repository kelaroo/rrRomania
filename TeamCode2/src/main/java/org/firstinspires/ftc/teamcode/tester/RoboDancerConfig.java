package org.firstinspires.ftc.teamcode.tester;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class RoboDancerConfig {
    public DcMotor rightFront;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor leftBack;

    public Servo Dr1;
    public Servo Dr2;
    public Servo Dr3;
    public Servo St1;
    public Servo St2;
    public Servo St3;
    public Servo cap;

    public RoboDancerConfig(HardwareMap hw) {
        rightBack = hw.get(DcMotor.class, "rightBack");
        rightFront = hw.get(DcMotor.class, "rightFront");
        leftBack = hw.get(DcMotor.class, "leftBack");
        leftFront = hw.get(DcMotor.class, "leftFront");
        List<DcMotor> lDriveMotors = Arrays.asList(rightFront, rightBack, leftBack, leftFront);

        for(DcMotor motor: lDriveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        cap = hw.get(Servo.class, "cap");
        Dr1 = hw.get(Servo.class, "dreapta1");
        Dr2 = hw.get(Servo.class, "dreapta2");
        Dr3 = hw.get(Servo.class, "dreapta3");
        St1 = hw.get(Servo.class, "stanga1");
        St2 = hw.get(Servo.class, "stanga2");
        St3 = hw.get(Servo.class, "stanga3");
    }

    public double clipPower(double power) {
        return (power > 1) ? 1 : ((power < -1) ? -1 : power);
    }
}
