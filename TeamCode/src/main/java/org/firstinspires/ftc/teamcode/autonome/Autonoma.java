package org.firstinspires.ftc.teamcode.autonome;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonoma extends LinearOpMode {
    Camera camera;
    SampleMecanumDrive drive;
    RRHardwareConfig sisteme;

    private final Camera.RingsDetectionPipeline.RingsNumber NONE = Camera.RingsDetectionPipeline.RingsNumber.NONE;
    private final Camera.RingsDetectionPipeline.RingsNumber ONE = Camera.RingsDetectionPipeline.RingsNumber.ONE;
    private final Camera.RingsDetectionPipeline.RingsNumber FOUR = Camera.RingsDetectionPipeline.RingsNumber.FOUR;

    final Pose2d startPose = new Pose2d(-63.0, -40.0, Math.toRadians(0.0));

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        sisteme = new RRHardwareConfig(hardwareMap);

        drive.setPoseEstimate(startPose);

        Camera.RingsDetectionPipeline.RingsNumber ringsNumber = Camera.RingsDetectionPipeline.getNumberOfRings();

        while(!isStarted()) {
            ringsNumber = Camera.RingsDetectionPipeline.getNumberOfRings();
            telemetry.addData("ringsNumber", ringsNumber);
            telemetry.update();
        }

        if(ringsNumber == NONE) { // A
            Trajectory trajA1 = drive.trajectoryBuilder(startPose)
                    .forward(60.0)
                    .addTemporalMarker(0.7, ()->{sisteme.lansat.setPower(LANSAT_POWER);})
                    .build();
            Trajectory trajA2 = drive.trajectoryBuilder(trajA1.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(90.0))))
                    .lineToConstantHeading(new Vector2d(3.0, -55.0))
                    .build();
            Trajectory trajA3 = drive.trajectoryBuilder(trajA2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-90.0))))
                    .lineToConstantHeading(new Vector2d(-40, -13.8))
                    .build();
            Trajectory trajA4 = drive.trajectoryBuilder(trajA3.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(90.0))))
                    .lineToConstantHeading(new Vector2d(22.0, -45.0))
                    .build();

            drive.followTrajectory(trajA1);

            drive.turn(Math.toRadians(-10.0));

            shoot(); shoot(); shoot();
            sisteme.lansat.setPower(0);

            drive.turn(Math.toRadians(100.0));
            sisteme.bratWobble.setPosition(BRAT_JOS);

            drive.followTrajectory(trajA2);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            sisteme.bratWobble.setPosition(BRAT_SUS);
            drive.turn(Math.toRadians(-90.0));

            drive.followTrajectory(trajA3);

            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(1000);
            sisteme.clawWobble.setPosition(CLAW_PRINS);

            drive.turn(Math.toRadians(90.0));

            drive.followTrajectory(trajA4);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

        } else if(ringsNumber == ONE) { // B

        } else { // C

        }
    }

    void shoot() {
        sisteme.impins.setPosition(IMPINS_FWD); sleep(250);
        sisteme.impins.setPosition(IMPINS_BWD); sleep(250);
    }
}
