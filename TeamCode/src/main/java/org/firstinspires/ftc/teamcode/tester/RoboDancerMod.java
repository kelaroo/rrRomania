package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.faraRR.HardwareConfig;

@Config
@TeleOp
public class RoboDancerMod extends OpMode {

    public static double dr1 = 0.83;
    public static double dr2 = 0;
    public static double dr3 = 0;
    public static double dr4 = 0;

    public static double st1 = 0;
    public static double st2 = 0;
    public static double st3 = 0;
    public static double st4 = 0;

    public static double cap = 0;


    RoboDancerConfig hw;
    @Override
    public void init() {
        hw = new RoboDancerConfig(hardwareMap);

        hw.St1.setPosition(st1);
        hw.St2.setPosition(st2);
        hw.St3.setPosition(st3);
        hw.St4.setPosition(st4);

        hw.Dr1.setPosition(dr1);
        hw.Dr2.setPosition(dr2);
        hw.Dr3.setPosition(dr3);
        hw.Dr4.setPosition(dr4);

        hw.cap.setPosition(cap);
    }

    @Override
    public void loop() {
        hw.St1.setPosition(st1);
        hw.St2.setPosition(st2);
        hw.St3.setPosition(st3);
        hw.St4.setPosition(st4);

        hw.Dr1.setPosition(dr1);
        hw.Dr2.setPosition(dr2);
        hw.Dr3.setPosition(dr3);
        hw.Dr4.setPosition(dr4);

        hw.cap.setPosition(cap);
    }
}
