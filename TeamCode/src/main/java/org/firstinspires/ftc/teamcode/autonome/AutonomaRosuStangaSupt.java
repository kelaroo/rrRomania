package org.firstinspires.ftc.teamcode.autonome;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BARA_PARCAT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BRAT_JOS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BRAT_PARCAT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BRAT_SUS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CLAW_LASAT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CLAW_PRINS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CUVA_JOS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CUVA_SUS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_BWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_FWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_RIGHT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_STATIONARY;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE3_SUCK;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_AUTO_A1;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_AUTO_B1;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_AUTO_B2;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_AUTO_C1;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_AUTO_C2;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomaRosuStangaSupt extends LinearOpMode {
    Camera camera;
    SampleMecanumDrive drive;
    RRHardwareConfig sisteme;

    private final Camera.RingsDetectionPipeline.RingsNumber NONE = Camera.RingsDetectionPipeline.RingsNumber.NONE;
    private final Camera.RingsDetectionPipeline.RingsNumber ONE = Camera.RingsDetectionPipeline.RingsNumber.ONE;
    private final Camera.RingsDetectionPipeline.RingsNumber FOUR = Camera.RingsDetectionPipeline.RingsNumber.FOUR;

    final Pose2d startPose = new Pose2d(-63.0, -20.0, Math.toRadians(0.0));

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
                    .lineTo(new Vector2d(-10.0, -35.0))
                    .build();
            Trajectory trajA2 = drive.trajectoryBuilder(trajA1.end())
                    .lineToLinearHeading(new Pose2d(10.0, -45.0, Math.toRadians(90.0)))
                    .build();
            Trajectory trajA3=drive.trajectoryBuilder(trajA2.end())
                    .forward(30.0)
                    .build();

            drive.followTrajectory(trajA1);
            drive.followTrajectory(trajA2);
            drive.followTrajectory(trajA3);

            PoseStorage.autoEndPose = drive.getPoseEstimate();

            while(opModeIsActive())
                ;

        } else if(ringsNumber == ONE) { // B

            Trajectory trajB1 = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-40.0,-30.0,Math.toRadians(-15.0)))
                    .build();
            Trajectory trajB2 = drive.trajectoryBuilder(trajB1.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-15.0))))
                    .forward(15.0)
                    .build();
            Trajectory trajB3 = drive.trajectoryBuilder(trajB2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(70.0))))
                    .lineToLinearHeading(new Pose2d(32.0,-43.0, Math.toRadians(180.0)))
                    .build();
            PoseStorage.autoEndPose = drive.getPoseEstimate();

            drive.followTrajectory(trajB1);

            //lanseaza

            drive.turn(Math.toRadians(-15.0));

            drive.followTrajectory(trajB2);

            //suge

            drive.turn(Math.toRadians(70.0));

            drive.followTrajectory(trajB3);

            while(opModeIsActive())
                ;

        } else { // C

            PoseStorage.autoEndPose = drive.getPoseEstimate();

            sisteme.bratWobble.setPosition(BRAT_SUS);

            while(opModeIsActive())
                continue;
        }
    }

    void shoot() {
        sisteme.impins.setPosition(IMPINS_FWD); sleep(250);
        sisteme.impins.setPosition(IMPINS_BWD); sleep(250);
    }

    void intakeOn() {
        sisteme.intake.setPower(1);
        sisteme.intake2.setPosition(INTAKE2_RIGHT);
        sisteme.intake3.setPower(INTAKE3_SUCK);
    }

    void intakeOff() {
        sisteme.intake.setPower(0);
        sisteme.intake2.setPosition(INTAKE2_STATIONARY);
        sisteme.intake3.setPower(0);
    }

    public TrajectoryBuilder myTrajectoryBuilder(Pose2d startPose, double maxVelo, double maxAccel){
        MinVelocityConstraint myVelConstraint = new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(maxAccel), new MecanumVelocityConstraint(maxVelo, DriveConstants.TRACK_WIDTH)));
        ProfileAccelerationConstraint myAccelConstraint = new ProfileAccelerationConstraint(maxAccel);
        return new TrajectoryBuilder(startPose, myVelConstraint, myAccelConstraint);
    }
}
