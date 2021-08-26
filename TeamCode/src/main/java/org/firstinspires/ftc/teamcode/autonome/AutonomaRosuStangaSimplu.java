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
import org.firstinspires.ftc.teamcode.systems.Cuva;
import org.firstinspires.ftc.teamcode.systems.Wobble;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_BWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_FWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_RIGHT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_STATIONARY;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE3_SUCK;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomaRosuStangaSimplu extends LinearOpMode {
    CameraDreapta camera;
    SampleMecanumDrive drive;
    RRHardwareConfig sisteme;

    private final CameraDreapta.RingsDetectionPipeline.RingsNumber NONE = CameraDreapta.RingsDetectionPipeline.RingsNumber.NONE;
    private final CameraDreapta.RingsDetectionPipeline.RingsNumber ONE = CameraDreapta.RingsDetectionPipeline.RingsNumber.ONE;
    private final CameraDreapta.RingsDetectionPipeline.RingsNumber FOUR = CameraDreapta.RingsDetectionPipeline.RingsNumber.FOUR;

    final Pose2d startPose = new Pose2d(-63.0, -17.0, 0.0); //81 cm // 28.5cm

    final double LANSAT_REDS_A1 = 1410;

    final double LANSAT_REDS_B1 = 1420;

    final double LANSAT_REDS_C1 = 1440;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new CameraDreapta(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        sisteme = new RRHardwareConfig(hardwareMap);

        drive.setPoseEstimate(startPose);

        CameraDreapta.RingsDetectionPipeline.RingsNumber ringsNumber = CameraDreapta.RingsDetectionPipeline.getNumberOfRings();

        while(!isStarted()) {
            ringsNumber = CameraDreapta.RingsDetectionPipeline.getNumberOfRings();
            telemetry.addData("ringsNumber", ringsNumber);
            telemetry.update();
        }

        /*new Thread(new Runnable() {
            @Override
            public void run() {
                Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

                while(opModeIsActive()) {
                    telemetry.addData("currVelo", sisteme.lansat.getVelocity());
                    telemetry.update();
                }
            }
        }).start();*/

        sisteme.lowerBratOprit();
        sisteme.baraOprit.setPosition(0.04);

        if(ringsNumber == NONE) { // A
            Trajectory trajA1 = drive.trajectoryBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(-5, -6))
                    .addTemporalMarker(0.7, ()->{
                        sisteme.lansat.setVelocity(LANSAT_REDS_A1);
                        sisteme.cuva.setPosition(Cuva.CUVA_SUS);
                    })
                    .build();
            Trajectory trajA2 = drive.trajectoryBuilder(trajA1.end())
                    .lineToLinearHeading(new Pose2d(30, -60.0, Math.toRadians(0.0)))
                    .build();
            Trajectory trajA3 = drive.trajectoryBuilder(trajA2.end())
                    .splineToLinearHeading(new Pose2d(50.0, -23, Math.toRadians(90.0)), Math.toRadians(180.0))
                    .build();
            Trajectory trajA4 = drive.trajectoryBuilder(trajA3.end())
                    .lineToLinearHeading(new Pose2d(3, -15))
                    .build();

            sleep(5000);

            drive.followTrajectory(trajA1);

            drive.turn(Math.toRadians(-22)); // trebuie modificat sus
            sleep(150); shoot(); sleep(150); shoot(); sleep(150); shoot();

            drive.followTrajectory(trajA2);


            sisteme.cuva.setPosition(Cuva.CUVA_JOS);
            intakeOn();

            sisteme.bratWobble.setPosition(Wobble.BRAT_JOS); sleep(200);
            sisteme.clawWobble.setPosition(Wobble.CLAW_LASAT); sleep(1000);
            sisteme.bratWobble.setPosition(Wobble.BRAT_SUS); sleep(200);

            drive.followTrajectory(trajA3);

            drive.followTrajectory(trajA4);

            while(opModeIsActive())
                ;

        } else if(ringsNumber == ONE) { // B

            Trajectory trajB1 = drive.trajectoryBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(-5, 0))
                    .addTemporalMarker(0.7, ()->{
                        sisteme.lansat.setVelocity(LANSAT_REDS_A1);
                        sisteme.cuva.setPosition(Cuva.CUVA_SUS);
                    })
                    .build();
            Trajectory trajB2 = drive.trajectoryBuilder(trajB1.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-26))))
                    .lineToLinearHeading(new Pose2d(35, -21.0, Math.toRadians(90.0)))
                    .build();
            Trajectory trajB3 = drive.trajectoryBuilder(trajB2.end())
                    //.lineToLinearHeading(new Pose2d(50.0, -30.0, Math.toRadians(30.0)))
                    .lineToLinearHeading(new Pose2d(3, 0))
                    .build();

            sleep(5000);

            drive.followTrajectory(trajB1);

            drive.turn(Math.toRadians(-26)); // trebuie modificat sus
            sleep(150); shoot(); sleep(150); shoot(); sleep(150); shoot();

            drive.followTrajectory(trajB2);


            sisteme.bratWobble.setPosition(Wobble.BRAT_JOS); sleep(200);
            sisteme.clawWobble.setPosition(Wobble.CLAW_LASAT); sleep(1000);
            sisteme.bratWobble.setPosition(Wobble.BRAT_SUS); sleep(200);

            drive.followTrajectory(trajB3);


            while(opModeIsActive())
                ;

        } else { // C
            Trajectory trajC1 = drive.trajectoryBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(-5, -10))
                    .addTemporalMarker(0.7, ()->{
                        sisteme.lansat.setVelocity(LANSAT_REDS_A1);
                        sisteme.cuva.setPosition(Cuva.CUVA_SUS);
                    })
                    .build();
            Trajectory trajC2 = drive.trajectoryBuilder(trajC1.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-22))))
                    .lineToLinearHeading(new Pose2d(55.0, -60, Math.toRadians(90.0)))
                    .build();
            Trajectory trajC3 = drive.trajectoryBuilder(trajC2.end())
                    .lineToLinearHeading(new Pose2d(3, -20))
                    .build();

            sleep(5000);

            drive.followTrajectory(trajC1);

            drive.turn(Math.toRadians(-22)); // trebuie modificat sus
            sleep(150); shoot(); sleep(150); shoot(); sleep(150); shoot();

            drive.followTrajectory(trajC2);

            sisteme.bratWobble.setPosition(Wobble.BRAT_JOS); sleep(200);
            sisteme.clawWobble.setPosition(Wobble.CLAW_LASAT); sleep(1000);
            sisteme.bratWobble.setPosition(Wobble.BRAT_SUS); sleep(200);

            drive.followTrajectory(trajC3);

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
