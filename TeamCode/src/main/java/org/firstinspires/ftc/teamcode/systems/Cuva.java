package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Cuva implements System {

    Robot robot;

    public static final double CUVA_SUS = 0.354;
    public static final double CUVA_JOS = 0.625;

    Servo cuva;

    public enum CuvaState {
        JOS, SUS
    }
    public CuvaState cuvaState = CuvaState.JOS;

    public Cuva(HardwareMap hw, Robot r) {
        cuva = hw.get(Servo.class, "cuva");
        robot = r;
    }

    @Override
    public void update() {
        if(robot.impins.impinsState == Impins.ImpinsState.AUTO)
            return;

        switch(cuvaState) {
            case JOS:
                cuva.setPosition(CUVA_JOS);
                break;
            case SUS:
                cuva.setPosition(CUVA_SUS);
                break;
        }
    }
}
