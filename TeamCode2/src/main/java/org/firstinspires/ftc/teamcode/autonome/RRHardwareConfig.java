package org.firstinspires.ftc.teamcode.autonome;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig2.*;

public class RRHardwareConfig {

    public DcMotor intake;
    public Servo intake2;
    public Servo intake3;

    public Servo bratWobble;
    public Servo clawWobble;

    public Servo cuva;
    public DcMotor lansat;
    public Servo impins;

    public RRHardwareConfig(HardwareMap hw) {
        intake2 = hw.get(Servo.class, "intake2");
        bratWobble = hw.get(Servo.class, "bratWobble");
        clawWobble = hw.get(Servo.class, "clawWobble");

        intake = hw.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lansat = hw.get(DcMotor.class, "lansat");
        lansat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cuva = hw.get(Servo.class, "cuva");
        impins = hw.get(Servo.class, "impins");
        intake3 = hw.get(Servo.class, "intake3");

        // Servo init
        cuva.setPosition(CUVA_JOS);
        bratWobble.setPosition(BRAT_SUS);
        clawWobble.setPosition(CLAW_PRINS);
        impins.setPosition(IMPINS_SECOND);

        /*bratOprit.setPosition(BRAT_OPRIT_EXT_AUTO);
        baraOprit.setPosition(BARA_OPRIT_EXT_AUTO);*/
    }
}
