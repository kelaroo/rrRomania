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
    public Servo St1;
    public Servo St2;
    public Servo St3;

    public RoboDancerConfig(HardwareMap hw){
        rightBack=hw.get(DcMotor.class,"rightBack");
        rightFront=hw.get(DcMotor.class,"rightFront");
        leftBack=hw.get(DcMotor.class,"leftBack");
        leftFront=hw.get(DcMotor.class,"leftFront");
        Dr1=hw.get(Servo.class,"Dr1");
        Dr2=hw.get(Servo.class,"Dr2");
        Dr3=hw.get(Servo.class,"Dr3");
        St1=hw.get(Servo.class,"St1");
        St2=hw.get(Servo.class,"St2");
        St3=hw.get(Servo.class,"St3");
    }
}
