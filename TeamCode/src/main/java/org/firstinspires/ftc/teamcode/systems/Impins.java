package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_BWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_FWD;

public class Impins implements System {

    Robot robot;

    public static final double IMPINS_FWD = 0;
    public static final double IMPINS_SECOND = 0.37;
    public static final double IMPINS_BWD = 0.27;

    Servo impins;

    public enum ImpinsPosition {
        BACK, SECOND, FWD
    }
    public enum ImpinsState {
        MANUAL, AUTO
    }
    public ImpinsPosition impinsPosition = ImpinsPosition.SECOND;
    public ImpinsState impinsState = ImpinsState.MANUAL;

    int autoShootCounter = 0;

    ElapsedTime autoShootTimer = null;

    public Impins(HardwareMap hw, Robot r) {
        impins = hw.get(Servo.class, "impins");
        robot = r;
    }

    @Override
    public void update() {
        if(robot.cuva.cuvaState == Cuva.CuvaState.JOS)
            return;

        switch(impinsState) {
            case AUTO:
                if(autoShootTimer == null)
                    autoShootTimer = new ElapsedTime();
                if(autoShootCounter < 3) {
                    if(autoShootTimer.milliseconds() < 250)
                        impinsPosition = ImpinsPosition.FWD;
                    else if(autoShootTimer.milliseconds() < 500)
                        impinsPosition = ImpinsPosition.BACK;
                    else {
                        autoShootCounter++;
                        autoShootTimer.reset();
                    }
                }
                else {
                    autoShootCounter = 0;
                    impinsState = ImpinsState.MANUAL;
                }
            break;
        }

        switch(impinsPosition) {
            case BACK:
                impins.setPosition(IMPINS_BWD);
                break;
            case SECOND:
                impins.setPosition(IMPINS_SECOND);
                break;
            case FWD:
                impins.setPosition(IMPINS_FWD);
                break;
        }
    }

    public void shoot() {
        ElapsedTime timer = new ElapsedTime();
        impins.setPosition(IMPINS_FWD);
        timer.reset();
        while(timer.milliseconds() < 500)
            ;

        impins.setPosition(IMPINS_BWD);
        timer.reset();
        while(timer.milliseconds() < 250)
            ;
    }
}
