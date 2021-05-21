package org.firstinspires.ftc.teamcode.autonome;

import android.view.textclassifier.TextClassifierEvent;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig2.*;

import org.apache.commons.math3.analysis.solvers.BracketedRealFieldUnivariateSolver;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonoma2 extends LinearOpMode {
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
            telemetry.addData("nrPixels", Camera.RingsDetectionPipeline.nrPixels);
            telemetry.update();
        }

        if(ringsNumber == NONE) { // A
            Trajectory trajA1 = drive.trajectoryBuilder(startPose)
                    .forward(60.0)
                    .addTemporalMarker(0.7, ()->{
                        sisteme.lansat.setPower(LANSAT_POWER-0.06);
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();
            Trajectory trajA2 = drive.trajectoryBuilder(trajA1.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(90.0))))
                    .lineToConstantHeading(new Vector2d(13.0, -55.0))
                    .build();
            Trajectory trajA3 = drive.trajectoryBuilder(trajA2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-90.0))))
                    .lineToConstantHeading(new Vector2d(-34.0, -29.0))
                    .build();
            Trajectory trajA4 = drive.trajectoryBuilder(trajA3.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(90.0))))
                    .lineToConstantHeading(new Vector2d(22.0, -45.0))
                    .build();
            Trajectory trajA5 = drive.trajectoryBuilder(trajA4.end())
                    .forward(20.0)
                    .build();

            drive.followTrajectory(trajA1);

            drive.turn(Math.toRadians(-5.0));

            sleep(350); shoot(); sleep(350); shoot(); sleep(350); shoot();
            sisteme.lansat.setPower(0);

            drive.turn(Math.toRadians(95.0));
            sisteme.bratWobble.setPosition(BRAT_JOS);

            drive.followTrajectory(trajA2);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            sisteme.bratWobble.setPosition(BRAT_SUS);
            drive.turn(Math.toRadians(-90.0));

            drive.followTrajectory(trajA3);

            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(1000);
            sisteme.clawWobble.setPosition(CLAW_PRINS); sleep(500);

            drive.turn(Math.toRadians(90.0));

            drive.followTrajectory(trajA4);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajA5);

            sisteme.bratWobble.setPosition(BRAT_SUS);
            sisteme.impins.setPosition(IMPINS_SECOND);
            sisteme.cuva.setPosition(CUVA_JOS);

            while(opModeIsActive())
                ;

        } else if(ringsNumber == ONE) { // B
            Trajectory trajB1 = drive.trajectoryBuilder(startPose)
                    .forward(22.0)
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setPower(LANSAT_POWER-0.05); // 0.023
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();
            Trajectory trajB12 = drive.trajectoryBuilder(trajB1.end())
                    .strafeLeft(8.0)
                    .build();
            Trajectory trajB2 = drive.trajectoryBuilder(trajB12.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-7.5))))
                    .forward(13.0)
                    .build();
            Trajectory trajB3 = drive.trajectoryBuilder(trajB2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(7.5))))
                    .lineToLinearHeading(new Pose2d(30.0, -43.0, Math.toRadians(180.0)))
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            Trajectory trajB4 = drive.trajectoryBuilder(trajB3.end())
                    .lineToLinearHeading(new Pose2d(-34.0, -34.0, Math.toRadians(0.0)))
                    .addTemporalMarker(0.5, ()->{sisteme.bratWobble.setPosition(BRAT_SUS);})
                    .build();
            Trajectory trajB5 = drive.trajectoryBuilder(trajB4.end())
                    .lineToLinearHeading(new Pose2d(25.0, -46.0, Math.toRadians(180.0)))
                    .build();
            Trajectory trajB6 = drive.trajectoryBuilder(trajB5.end())
                    .forward(10.0)
                    .build();

            drive.followTrajectory(trajB1);

            drive.followTrajectory(trajB12);

            drive.turn(Math.toRadians(-7.5));
            sleep(500);
            shoot(); sleep(400); shoot(); sleep(400); shoot(); sisteme.lansat.setPower(LANSAT_POWER+0.04); // 0.065
            sisteme.cuva.setPosition(CUVA_JOS);
            intakeOn();

            drive.followTrajectory(trajB2);

            intakeOff();
            sisteme.lansat.setPower(LANSAT_POWER+0.04);
            sisteme.cuva.setPosition(CUVA_SUS); sleep(800);
            shoot();
            sisteme.lansat.setPower(0);

            drive.turn(Math.toRadians(7.5));

            drive.followTrajectory(trajB3);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajB4);

            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(1000);
            sisteme.clawWobble.setPosition(CLAW_PRINS);

            drive.followTrajectory(trajB5);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajB6);

            sisteme.bratWobble.setPosition(BRAT_SUS);
            while(opModeIsActive())
                ;

        } else { // C
            /*sisteme.bratOprit.setPosition(BRAT_OPRIT_EXT_AUTO);
            sisteme.baraOprit.setPosition(BARA_OPRIT_EXT_AUTO);*/

            Trajectory trajC1 = myTrajectoryBuilder(startPose, 60, 60)
                    .splineToConstantHeading(new Vector2d(-28.0, -34.0), Math.toRadians(0.0))
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setPower(LANSAT_POWER);
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();

            Trajectory trajC2 = myTrajectoryBuilder(trajC1.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-0.0))), 30, 20)
                    .forward(16)
                    .build();

            Trajectory trajC3 = drive.trajectoryBuilder(trajC2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(0.0))))
                    .forward(5.0)
                    .build();

            Trajectory trajC4 = myTrajectoryBuilder(trajC3.end(), 60, 60)
                    .lineToLinearHeading(new Pose2d(65.0, -76.0, Math.toRadians(180.0)))
                    .addTemporalMarker(0.5, ()->{intakeOff();})
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            /*Trajectory trajC5 = myTrajectoryBuilder(trajC4.end(), 60, 60)
                    .lineToLinearHeading(new Pose2d(-38.5, -16.5, Math.toRadians(0.0)))
                    .addTemporalMarker(0.5, ()->{sisteme.bratWobble.setPosition(BRAT_SUS);})
                    .build();
            Trajectory trajC6 = myTrajectoryBuilder(trajC5.end(), 60, 60)
                    .lineToLinearHeading(new Pose2d(49.0, -80.0, Math.toRadians(180.0)))
                    .addTemporalMarker(0.5, ()->{sisteme.bratWobble.setPosition(BRAT_SUS);})
                    .build();
            Trajectory trajC7 = myTrajectoryBuilder(trajC6.end(), 60, 60)
                    .forward(42.0)
                    .build();*/
            Trajectory trajC5 = myTrajectoryBuilder(trajC4.end(), 60.0, 60.0)
                    .lineToConstantHeading(new Vector2d(-24.5, -19.7))
                    .build();
            Trajectory trajC51 = myTrajectoryBuilder(trajC5.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(183))), 60.0, 60.0)
                    .back(17.0)
                    .build();
            Trajectory trajC6 = myTrajectoryBuilder(trajC51.end(), 60.0, 60.0)
                    .lineToConstantHeading(new Vector2d(46.0, -75.0))
                    .build();
            Trajectory trajC7 = myTrajectoryBuilder(trajC6.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-180.0))), 60.0, 60.0)
                    .forward(30.0)
                    .build();


            drive.followTrajectory(trajC1);
            drive.turn(Math.toRadians(-7.0));
            shoot();sleep(200);shoot(); sleep(200); shoot();
            sisteme.cuva.setPosition(CUVA_JOS);
            intakeOn();
            sisteme.lansat.setPower(LANSAT_POWER-0.1);

            drive.turn(Math.toRadians(7.0));

            drive.followTrajectory(trajC2);
            //sleep(600);
            sisteme.cuva.setPosition(CUVA_SUS);
            //intakeOff();
            shoot();shoot();shoot();
            sisteme.cuva.setPosition(CUVA_JOS);
            //drive.turn(Math.toRadians(9));
            //intakeOn();
            sisteme.lansat.setPower(0);

            drive.followTrajectory(trajC3);
            drive.followTrajectory(trajC4);
            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajC5);

            drive.turn(Math.toRadians(183));
            drive.followTrajectory(trajC51);

            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(350);
            sisteme.clawWobble.setPosition(CLAW_PRINS);

            drive.followTrajectory(trajC6);
            drive.turn(Math.toRadians(-180.0));

            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(370);
            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajC7);

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
        sisteme.intake.setPower(INTAKE_SUCK);
        sisteme.intake2.setPosition(INTAKE2_RIGHT);
        sisteme.intake3.setPosition(INTAKE3_RIGHT);
    }

    void intakeOff() {
        sisteme.intake.setPower(0);
        sisteme.intake2.setPosition(INTAKE2_STATIONARY);
        sisteme.intake3.setPosition(INTAKE3_STATIONARY);
    }

    public TrajectoryBuilder myTrajectoryBuilder(Pose2d startPose, double maxVelo, double maxAccel){
        MinVelocityConstraint myVelConstraint = new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(maxAccel), new MecanumVelocityConstraint(maxVelo, DriveConstants.TRACK_WIDTH)));
        ProfileAccelerationConstraint myAccelConstraint = new ProfileAccelerationConstraint(maxAccel);
        return new TrajectoryBuilder(startPose, myVelConstraint, myAccelConstraint);
    }
}
