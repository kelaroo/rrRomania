package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonome.RRHardwareConfig;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CUVA_SUS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_BWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_FWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_AUTO_PS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_POWER_AUTO;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_SPEED_PS;

@Disabled
@Autonomous
public class AutonomaNebuna extends LinearOpMode {

    SampleMecanumDrive drive;
    RRHardwareConfig sisteme;

    final Pose2d startPose = new Pose2d(-63.0, -40.0, Math.toRadians(0.0));

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        sisteme = new RRHardwareConfig(hardwareMap);

        drive.setPoseEstimate(startPose);

        waitForStart();

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(5.0)
                .addTemporalMarker(0.5, ()->{ sisteme.cuva.setPosition(CUVA_SUS); })
                .build();

        sisteme.lansat.setVelocity(LANSAT_AUTO_PS);
        drive.followTrajectory(traj1);
        sleep(2000);
        shoot();

        drive.turn(Math.toRadians(4.5));
        shoot();
        drive.turn(Math.toRadians(4.0));
        shoot();

    }

    void shoot() {
        sisteme.impins.setPosition(IMPINS_FWD); sleep(250);
        sisteme.impins.setPosition(IMPINS_BWD); sleep(250);
    }
}
