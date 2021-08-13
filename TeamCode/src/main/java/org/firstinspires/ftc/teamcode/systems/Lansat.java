package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lansat implements System {

    //public static PIDFCoefficients lansatCoeff = new PIDFCoefficients(30, 0.0, 10, 13.45);
    public static PIDFCoefficients lansatCoeff = new PIDFCoefficients(30, 0.0, 20, 12.6);
    public static double LANSAT_POWER_AUTO = 0.65;
    public static double LANSAT_SPEED = 1450;
    public static double LANSAT_SPEED_PS = 1270; // 1270

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
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();


        switch(lansatState) {
            case IDLE:
                lansat.setVelocity(0);
                telemetry.addData("targetVelo", 0);
                break;
            case HIGH_GOAL:
                lansat.setVelocity(LANSAT_SPEED);
                telemetry.addData("targetVelo", LANSAT_SPEED);
                break;
            case POWERSHOTS:
                lansat.setVelocity(LANSAT_SPEED_PS);
                telemetry.addData("targetVelo", LANSAT_SPEED_PS);
                break;
        }

        telemetry.addData("currVelo", lansat.getVelocity());
        telemetry.update();
    }
}
