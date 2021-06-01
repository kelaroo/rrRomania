package org.firstinspires.ftc.teamcode.autonome;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig2.*;

public class RRHardwareConfig {

    public DcMotor intake;
    public Servo intake2;
    //public Servo intake3;

    public DcMotor intake3;

    public Servo bratWobble;
    public Servo clawWobble;

    public Servo cuva;
    public DcMotorEx lansat;
    public Servo impins;

    public Servo bratOprit;
    public Servo baraOprit;

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
        intake3.setDirection(DcMotorSimple.Direction.REVERSE);

        bratOprit = hw.get(Servo.class, "bratOprit");
        baraOprit = hw.get(Servo.class, "baraOprit");

        // Servo init
        cuva.setPosition(CUVA_JOS);
        bratWobble.setPosition(BRAT_SUS);
        clawWobble.setPosition(CLAW_PRINS);
        impins.setPosition(IMPINS_SECOND);
        bratOprit.setPosition(BRAT_OPRIT_INIT);


        /*bratOprit.setPosition(BRAT_OPRIT_EXT_AUTO);
        baraOprit.setPosition(BARA_OPRIT_EXT_AUTO);*/
    }
}
