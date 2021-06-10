package org.firstinspires.ftc.teamcode.systems.threads;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Impins;

public class OneShotOnly implements Runnable {

    Impins impins;

    void shoot(double time) {
        ElapsedTime timer = new ElapsedTime();
        impins.impingeFwd();
        while(timer.milliseconds() < time)
            ;
        impins.impingeBwd();
        timer.reset();
        while(timer.milliseconds() < 200)
            ;
    }

    public OneShotOnly() {
        impins = Impins.getInstance();
    }

    @Override
    public void run() {
        impins.state = Impins.State.SHOOTING;

        shoot(500);
        shoot(400);
        shoot(250);

        impins.state = Impins.State.IDLE;
    }
}
