package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Lansat {

    // Powers
    public static final PIDFCoefficients lansatCoeff = new PIDFCoefficients(25, 0.0, 10, 13.45);
    public static final double LANSAT_SPEED = 1450;
    public static final double LANSAT_SPEED_PS = 1220;
    public static final double LANSAT_POWER_AUTO = 0.65;

    // Hardware
    DcMotorEx lansat;

    // Singleton
    private static Lansat instance = null;

    private Lansat(HardwareMap hw) {
        lansat = hw.get(DcMotorEx.class, "odoCenter");
        lansat.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lansatCoeff);
    }

    public static Lansat getInstance(HardwareMap hw) {
        if(instance == null)
            instance = new Lansat(hw);
        return instance;
    }

    // Functionality
    public void lansatHigh() { lansat.setVelocity(LANSAT_SPEED); }
    public void lansatPS() { lansat.setVelocity(LANSAT_SPEED_PS); }
    public void lansatStop() { lansat.setPower(0); }
}
