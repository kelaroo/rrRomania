package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_BWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_FWD;

@Config
public class Impins implements System {

    Robot robot;

    public static double IMPINS_FWD = 0;
    public static double IMPINS_SECOND = 0.37;
    public static double IMPINS_BWD = 0.27;

    public static int AUTO_FWD = 200;
    public static int AUTO_BWD = 230;

    public Servo impins;

    public enum ImpinsPosition {
        BACK, SECOND, FWD
    }
    public enum ImpinsState {
        MANUAL, NON_MANUAL, AUTO
    }
    public ImpinsPosition impinsPosition = ImpinsPosition.SECOND;
    public ImpinsState impinsState = ImpinsState.MANUAL;

    int autoShootCounter = 0;

    ElapsedTime autoShootTimer = null;

    Thread threadThreeShots = null;

    public Impins(HardwareMap hw, Robot r) {
        impins = hw.get(Servo.class, "impins");
        robot = r;
    }

    @Override
    public void update() {
        if(robot.cuva.cuvaState == Cuva.CuvaState.JOS || impinsState == ImpinsState.NON_MANUAL)
            return;

        robot.telemetry.addData("impinsPos", impinsPosition);
        robot.telemetry.update();

        switch(impinsState) {
            case AUTO:
                if(threadThreeShots == null) {
                    threadThreeShots = new Thread(new AutoThreeShot());
                    threadThreeShots.start();
                }
                return;
            default:
                threadThreeShots = null;
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
        while (timer.milliseconds() < 500)
            ;

        impins.setPosition(IMPINS_BWD);
        timer.reset();
        while (timer.milliseconds() < 250)
            ;
    }

    class AutoThreeShot implements Runnable {

        void waitTimer(int milliseconds) {
            ElapsedTime timer = new ElapsedTime();
            while(timer.milliseconds() < milliseconds)
                ;
        }

        @Override
        public void run() {
            impinsState = ImpinsState.AUTO;

            for(int i = 0; i < 3; i++) {
                impinsPosition = ImpinsPosition.FWD;
                impins.setPosition(IMPINS_FWD);

                waitTimer(AUTO_FWD);

                impinsPosition = ImpinsPosition.BACK;
                impins.setPosition(IMPINS_BWD);

                waitTimer(AUTO_BWD);
            }

            impinsState = ImpinsState.MANUAL;
        }
    }
}
