package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bars {

    // Powers bratOprit + baraOprit
    public static final double BRAT_OPRIT_INT = 0.61;
    public static final double BRAT_OPRIT_EXT = 0.77;
    public static final double BRAT_OPRIT_EXT_AUTO = 0.91;
    public static final double BARA_OPRIT_INT = 0.65;
    public static final double BARA_OPRIT_EXT = 0.005;
    public static final double BARA_OPRIT_EXT_AUTO = 0.25;
    public static final double BRAT_PARCAT = 0.93;
    public static final double BARA_PARCAT = 0.005;

    // Powers baraS
    public static final double BARAS_EXT = 0.75;
    public static final double BARAS_INT = 0.45;
    public static final double BARAS_INIT = 0.3;

    // Hardware
    Servo bratOprit;
    Servo baraOprit;
    Servo baraS;

    // Singleton
    private static Bars instance = null;

    private Bars(HardwareMap hw) {
        bratOprit = hw.get(Servo.class, "bratOprit");
        baraOprit = hw.get(Servo.class, "baraOprit");
        baraS = hw.get(Servo.class, "baraOprit");
    }

    public Bars getInstance(HardwareMap hw) {
        if(instance == null)
            instance = new Bars(hw);
        return instance;
    }

    // Functionality
    public void init() {
        baraS.setPosition(BARAS_INIT);
        baraOprit.setPosition(BARA_OPRIT_INT);
        bratOprit.setPosition(BRAT_OPRIT_INT);
    }

    public void baraInterior() { baraS.setPosition(BARAS_INT); }
    public void baraExterior() { baraS.setPosition(BARAS_EXT); }

    public void bratInterior() {
        baraOprit.setPosition(BARA_OPRIT_INT);
        bratOprit.setPosition(BRAT_OPRIT_INT);
    }
    public void bratExterior() {
        baraOprit.setPosition(BARA_OPRIT_EXT);
        bratOprit.setPosition(BRAT_OPRIT_EXT);
    }
}
