package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Lansat implements System {

    public static PIDFCoefficients lansatCoeff = new PIDFCoefficients(30, 0.0, 10, 13.45);
    public static double LANSAT_POWER_AUTO = 0.65;
    public static double LANSAT_SPEED = 1450;
    public static double LANSAT_SPEED_PS = 1270;

    DcMotorEx lansat;

    public enum LansatState {
        IDLE,
        HIGH_GOAL,
        POWERSHOTS
    }
    public LansatState lansatState = LansatState.IDLE;

    public Lansat(HardwareMap hw) {
        lansat = hw.get(DcMotorEx.class, "lansat");
        lansat.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lansatCoeff);
    }

    @Override
    public void update() {
        switch(lansatState) {
            case IDLE:
                lansat.setVelocity(0);
                break;
            case HIGH_GOAL:
                lansat.setVelocity(LANSAT_SPEED);
                break;
            case POWERSHOTS:
                lansat.setVelocity(LANSAT_SPEED_PS);
                break;
        }
    }
}
