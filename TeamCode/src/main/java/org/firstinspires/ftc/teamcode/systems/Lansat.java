package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Lansat implements System {

    public static final PIDFCoefficients lansatCoeff = new PIDFCoefficients(25, 0.0, 10, 12.8);
    public static final double LANSAT_POWER_AUTO = 0.65;
    public static final double LANSAT_SPEED = 1450;
    public static final double LANSAT_SPEED_PS = 1270;

    DcMotorEx lansat;

    public enum LansatState {
        IDLE(0),
        HIGH_GOAL(1450),
        POWERSHOTS(1270);

        int speed;

        LansatState(int s) {
            speed = s;
        }
    }
    public LansatState lansatState = LansatState.IDLE;

    public Lansat(HardwareMap hw) {
        lansat = hw.get(DcMotorEx.class, "lansat");
        lansat.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lansatCoeff);
    }

    @Override
    public void update() {
        lansat.setVelocity(lansatState.speed);
    }
}
