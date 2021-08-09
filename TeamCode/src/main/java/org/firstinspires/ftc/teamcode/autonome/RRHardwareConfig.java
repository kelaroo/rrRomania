package org.firstinspires.ftc.teamcode.autonome;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.*;

import org.firstinspires.ftc.teamcode.faraRR.HardwareConfig;
import org.firstinspires.ftc.teamcode.systems.BaraOprit;

public class RRHardwareConfig {

    public DcMotor intake;
    public Servo intake2;
    public DcMotor intake3;

    public Servo bratWobble;
    public Servo clawWobble;

    public Servo cuva;
    public DcMotorEx lansat;
    public Servo impins;

    public Servo baraS;
    public Servo baraD;

    public Servo bratOprit; // ch 5
    public Servo baraOprit; // eh 3

    public RRHardwareConfig(HardwareMap hw) {
        intake2 = hw.get(Servo.class, "intake2");
        bratWobble = hw.get(Servo.class, "bratWobble");
        clawWobble = hw.get(Servo.class, "clawWobble");

        intake = hw.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lansat = hw.get(DcMotorEx.class, "lansat");
        lansat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lansat.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lansatCoeff);

        cuva = hw.get(Servo.class, "cuva");
        impins = hw.get(Servo.class, "impins");
        intake3 = hw.get(DcMotor.class, "intake3");

        baraD = hw.get(Servo.class, "baraD");
        baraS = hw.get(Servo.class, "baraS");

        bratOprit = hw.get(Servo.class, "bratOprit");
        baraOprit = hw.get(Servo.class, "baraOprit");

        // Servo init
        cuva.setPosition(CUVA_JOS);
        bratWobble.setPosition(BRAT_SUS);
        clawWobble.setPosition(CLAW_PRINS);
        impins.setPosition(IMPINS_BWD);

        bratOprit.setPosition(BRAT_OPRIT_INT);
        baraOprit.setPosition(BARA_OPRIT_INT);

        baraS.setPosition(BARAS_INIT);

        bratOprit.setPosition(BaraOprit.BRAT_OPRIT_INIT);
        baraOprit.setPosition(BaraOprit.BARA_OPRIT_INIT);

        /*bratOprit.setPosition(BRAT_OPRIT_EXT_AUTO);
        baraOprit.setPosition(BARA_OPRIT_EXT_AUTO);*/
    }
}
