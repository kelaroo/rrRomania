package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Singleton;

public class Wobble {

    // Powers
    public static final double BRAT_SUS = 0.85;
    public static final double BRAT_JOS = 0.25;
    public static final double CLAW_PRINS = 0.48;
    public static final double CLAW_LASAT = 0.98;

    // Hardware
    Servo bratWobble;
    Servo clawWobble;

    // Singleton
    private static Wobble instance = null;

    private Wobble(HardwareMap hw) {
        bratWobble = hw.get(Servo.class, "bratWobble");
        clawWobble = hw.get(Servo.class, "clawWobble");
    }

    public Wobble getInstance(HardwareMap hw) {
        if(instance == null)
            instance = new Wobble(hw);
        return instance;
    }

    // Functionality
    public void bratSus() { bratWobble.setPosition(BRAT_SUS); }
    public void bratJos() { bratWobble.setPosition(BRAT_JOS); }

    public void setClawPrins() { clawWobble.setPosition(CLAW_PRINS); }
    public void setClawLasat() { clawWobble.setPosition(CLAW_LASAT); }

}
