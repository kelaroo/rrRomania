package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Impins {

    // Powers
    public static final double IMPINS_FWD = 0.00; // 0
    public static final double IMPINS_SECOND = 0.37; // 0.3
    public static final double IMPINS_BWD = 0.27; // 0.25

    // States
    public enum State {
        SHOOTING, IDLE
    }
    public State state = State.IDLE;

    // Hardware
    Servo impins;

    // Singleton
    public static Impins instance = null;

    private Impins(HardwareMap hw) {
        impins = hw.get(Servo.class, "impins");
    }

    public static void createInstance(HardwareMap hw) {
        instance = new Impins(hw);
    }

    public static Impins getInstance() {
        return instance;
    }

    // Functionality
    public void impingeFwd() { impins.setPosition(IMPINS_FWD); }
    public void impingeBwd() { impins.setPosition(IMPINS_BWD); }
    public void impingeSecond() { impins.setPosition(IMPINS_SECOND); }
}
