package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble implements System {

    public static final double BRAT_SUS = 0.25;
    public static final double BRAT_JOS = 0.655;
    public static final double CLAW_PRINS = 0.48;
    public static final double CLAW_LASAT = 0.98;

    Servo brat;
    Servo claw;

    public enum BratState {
        JOS, SUS
    }
    public enum ClawState {
        PRINS, LASAT
    }
    public BratState bratState = BratState.SUS;
    public ClawState clawState = ClawState.PRINS;

    public Wobble(HardwareMap hw) {
        brat = hw.get(Servo.class, "bratWobble");
        claw = hw.get(Servo.class, "clawWobble");
    }

    @Override
    public void update() {
        switch(bratState) {
            case JOS:
                brat.setPosition(BRAT_JOS);
                break;
            case SUS:
                brat.setPosition(BRAT_SUS);
                break;
        }

        switch(clawState) {
            case PRINS:
                claw.setPosition(CLAW_PRINS);
                break;
            case LASAT:
                claw.setPosition(CLAW_LASAT);
                break;
        }
    }
}
