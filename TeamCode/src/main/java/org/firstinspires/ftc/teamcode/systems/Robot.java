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
    List<System> lSystems;

    public SampleMecanumDrive drive;

    Telemetry telemetry;

    public enum RobotSpeed {
        LOW(0.8),
        HIGH(0.5);

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
        lSystems = Arrays.asList(intake, cuva, lansat, impins, wobble);

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
                if(tMoveToPS == null)
                    tMoveToPS = new Thread(new MoveToPS());
                break;
        }
        telemetry.addData("RobotState", robotState);

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

        @Override
        public void run() {
            intake.intakeState = Intake.IntakeState.SUGE;

            waitTimer(100);

            cuva.cuvaState = Cuva.CuvaState.SUS;
            intake.intakeState = Intake.IntakeState.OFF;

            waitTimer(1000);

            impins.shoot();

            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(12.5) // 11 in
                    .addDisplacementMarker(()->{impins.shoot();})
                    .build();
            drive.followTrajectory(traj);

            traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(11.5) // 11 in
                    .addDisplacementMarker(()->{impins.shoot();})
                    .build();
            drive.followTrajectory(traj);

            robotState = RobotState.MANUAL;
        }
    }
}
