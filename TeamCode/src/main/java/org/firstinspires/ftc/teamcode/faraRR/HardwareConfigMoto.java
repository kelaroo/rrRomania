package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareConfigMoto {
    public DcMotor rightFront;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor leftBack;

    public DcMotor intake1;
    public DcMotor intake2;
    public Servo intake;

    public DcMotor launcher;
    public Servo cuva;
    public Servo impins;

    public Servo clawWobble;
    public Servo bratWobble;
    public HardwareConfigMoto(HardwareMap hw){
        rightFront=hw.get(DcMotor.class,"right_front");
        leftFront=hw.get(DcMotor.class,"left_front");
        rightBack=hw.get(DcMotor.class,"right_back");
        leftBack=hw.get(DcMotor.class,"left_back");

        intake1=hw.get(DcMotor.class,"intake");
        intake2=hw.get(DcMotor.class,"intake3");
        intake=hw.get(Servo.class,"intake2");

        cuva=hw.get(Servo.class,"cuva");
        impins=hw.get(Servo.class,"impins");
        launcher=hw.get(DcMotor.class,"lansat");

        clawWobble=hw.get(Servo.class,"clawWobble");
        bratWobble=hw.get(Servo.class,"bratWobble");
    }
}
