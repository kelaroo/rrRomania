package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.faraRR.TwoDriver;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CUVA_SUS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_FWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_RIGHT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_STATIONARY;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE3_SUCK;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE_SUCK;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_SPEED_PS;

public class Robot {

    public Intake intake;
    public Cuva cuva;
    public Lansat lansat;
    public Impins impins;
    public Wobble wobble;
    public BaraOprit baraOprit;
    List<System> lSystems;

    public SampleMecanumDrive drive;

    Telemetry telemetry;

    public enum RobotSpeed {
        LOW(0.3),
        HIGH(0.8);

        public double coeff;

        RobotSpeed(double coeff) {
            this.coeff = coeff;
        }
    }
    public RobotSpeed robotSpeed = RobotSpeed.HIGH;

    public enum RobotState {
        MANUAL, MOVE_TO_PS
    }
    public RobotState robotState = RobotState.MANUAL;

    Thread tMoveToPS = null;

    public Robot(HardwareMap hw, Telemetry t) {
        intake = new Intake(hw, this);
        cuva = new Cuva(hw, this);
        lansat = new Lansat(hw);
        impins = new Impins(hw, this);
        wobble = new Wobble(hw);
        baraOprit = new BaraOprit(hw);
        lSystems = Arrays.asList(intake, cuva, lansat, impins, wobble, baraOprit);

        drive = new SampleMecanumDrive(hw);

        telemetry = t;
    }

    public void update() {
        drive.update();

        switch(robotState) {
            case MANUAL:
                tMoveToPS = null;
                break;
            case MOVE_TO_PS:
                if(tMoveToPS == null) {
                    tMoveToPS = new Thread(new MoveToPS());
                    tMoveToPS.start();
                }
                break;
        }

        for(System system: lSystems)
            system.update();
    }

    // TeleOP autonomous
    class MoveToPS implements Runnable {

        void waitTimer(int time) {
            ElapsedTime timer = new ElapsedTime();
            while(timer.milliseconds() < time)
                ;
        }

        void shoot(int time) {
            impins.impinsPosition = Impins.ImpinsPosition.FWD;
            impins.impins.setPosition(impins.IMPINS_FWD);
            waitTimer(time);

            impins.impinsPosition = Impins.ImpinsPosition.BACK;
            impins.impins.setPosition(impins.IMPINS_BWD);
            waitTimer(250);
        }

        @Override
        public void run() {
            impins.impinsState = Impins.ImpinsState.NON_MANUAL;

            intake.intakeState = Intake.IntakeState.SUGE;
            lansat.lansatState = Lansat.LansatState.POWERSHOTS;

            waitTimer(100);

            cuva.cuvaState = Cuva.CuvaState.SUS;
            intake.intakeState = Intake.IntakeState.OFF;

            waitTimer(1000);

            shoot(250);

            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(12.5) // 11 in
                    .addDisplacementMarker(()->{shoot(250);})
                    .build();
            drive.followTrajectory(traj);

            traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(11.5) // 11 in
                    .addDisplacementMarker(()->{shoot(250);})
                    .build();
            drive.followTrajectory(traj);

            lansat.lansatState = Lansat.LansatState.IDLE;
            impins.impinsState = Impins.ImpinsState.MANUAL;
            robotState = RobotState.MANUAL;
        }
    }
}
