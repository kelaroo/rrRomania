package org.firstinspires.ftc.teamcode.autonome;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.*;

import org.firstinspires.ftc.teamcode.faraRR.HardwareConfig;

public class RRHardwareConfig {

    public DcMotor intake;
    public Servo intake2;
    public Servo intake3;

    public Servo bratWobble;
    public Servo clawWobble;

    public Servo cuva;
    public DcMotor lansat;
    public Servo impins;

    public Servo baraS;
    public Servo baraD;

    public Servo bratOprit; // ch 5
    public Servo baraOprit; // eh 3

    public RRHardwareConfig(HardwareMap hw) {
        intake2 = hw.get(Servo.class, "intake2");
        bratWobble = hw.get(Servo.class, "bratWobble");
        clawWobble = hw.get(Servo.class, "clawWobble");

        intake = hw.get(DcMotor.class, "odoRight");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lansat = hw.get(DcMotor.class, "odoCenter");
        lansat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cuva = hw.get(Servo.class, "cuva");
        impins = hw.get(Servo.class, "impins");
        intake3 = hw.get(Servo.class, "intake3");

        baraD = hw.get(Servo.class, "baraD");
        baraS = hw.get(Servo.class, "baraS");

        bratOprit = hw.get(Servo.class, "bratOprit");
        baraOprit = hw.get(Servo.class, "baraOprit");

        // Servo init
        cuva.setPosition(CUVA_JOS);
        bratWobble.setPosition(BRAT_SUS);
        clawWobble.setPosition(CLAW_PRINS);
        impins.setPosition(IMPINS_BWD);

        /*bratOprit.setPosition(BRAT_OPRIT_EXT_AUTO);
        baraOprit.setPosition(BARA_OPRIT_EXT_AUTO);*/
    }
}
