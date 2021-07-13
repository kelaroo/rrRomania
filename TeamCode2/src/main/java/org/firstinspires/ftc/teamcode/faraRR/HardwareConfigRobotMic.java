package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public class HardwareConfigRobotMic {
    public DcMotor rightFront;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor leftBack;

    public DcMotor fan;

    public Servo ashTray;

    public HardwareConfigRobotMic(HardwareMap hw) {
        rightFront=hw.get(DcMotor.class, "rightFront");
        leftFront=hw.get(DcMotor.class, "leftFront");
        rightBack=hw.get(DcMotor.class, "rightBack");
        leftBack=hw.get(DcMotor.class, "leftBack");

        fan=hw.get(DcMotor.class, "fan");

        ashTray=hw.get(Servo.class, "ashTray");
    }
}
