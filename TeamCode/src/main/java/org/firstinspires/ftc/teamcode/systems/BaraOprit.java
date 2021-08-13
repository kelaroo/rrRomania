package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class BaraOprit implements System {

    public static double BRAT_OPRIT_INIT = 0.678;
    public static double BRAT_OPRIT_INT = 0.69;
    public static double BRAT_OPRIT_EXT = 0.82;
    public static double BRAT_OPRIT_AUTO_C = 0.765;

    public static double BARA_OPRIT_INIT = 0.675;
    public static double BARA_OPRIT_INT = 0.6;
    public static double BARA_OPRIT_EXT = 0.14;
    public static double BARA_OPRIT_AUTO_C = 0.05;

    Servo bratOprit;
    Servo baraOprit;

    public enum OpritState {
        INIT, INT, EXT
    }
    public OpritState opritState = OpritState.INT;

    public BaraOprit(HardwareMap hw) {
        bratOprit = hw.get(Servo.class, "bratOprit");
        baraOprit = hw.get(Servo.class, "baraOprit");

        update();
    }

    @Override
    public void update() {
        switch(opritState) {
            case INIT:
                bratOprit.setPosition(BRAT_OPRIT_INIT);
                baraOprit.setPosition(BARA_OPRIT_INIT);
                break;
            case INT:
                bratOprit.setPosition(BRAT_OPRIT_INT);
                baraOprit.setPosition(BARA_OPRIT_INT);
                break;
            case EXT:
                bratOprit.setPosition(BRAT_OPRIT_EXT);
                baraOprit.setPosition(BARA_OPRIT_EXT);
                break;
        }
    }
}
