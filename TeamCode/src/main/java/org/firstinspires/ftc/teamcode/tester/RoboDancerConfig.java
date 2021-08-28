package org.firstinspires.ftc.teamcode.tester;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RoboDancerConfig {
    public DcMotor rightFront;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor leftBack;
    public Servo Dr1;
    public Servo Dr2;
    public Servo Dr3;
    public Servo Dr4;

    public Servo St1;
    public Servo St2;
    public Servo St3;
    public Servo St4;

    public Servo cap;

    public RoboDancerConfig(HardwareMap hw){
        rightBack=hw.get(DcMotor.class,"rightBack");
        rightFront=hw.get(DcMotor.class,"rightFront");
        leftBack=hw.get(DcMotor.class,"leftBack");
        leftFront=hw.get(DcMotor.class,"leftFront");

        Dr1=hw.get(Servo.class,"Dr1");
        Dr2=hw.get(Servo.class,"Dr2");
        Dr3=hw.get(Servo.class,"Dr3");
        Dr4=hw.get(Servo.class, "Dr4");

        St1=hw.get(Servo.class,"St1");
        St1.setDirection(Servo.Direction.REVERSE);
        St2=hw.get(Servo.class,"St2");
        St2.setDirection(Servo.Direction.REVERSE);
        St3=hw.get(Servo.class,"St3");
        St3.setDirection(Servo.Direction.REVERSE);
        St4=hw.get(Servo.class, "St4");
        St4.setDirection(Servo.Direction.REVERSE);

        cap = hw.get(Servo.class, "cap");
    }
}
