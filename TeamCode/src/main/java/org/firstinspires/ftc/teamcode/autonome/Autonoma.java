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

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.*;

import org.apache.commons.math3.analysis.solvers.BracketedRealFieldUnivariateSolver;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

import java.util.Arrays;

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
            /*Trajectory trajA1 = drive.trajectoryBuilder(startPose)
                    .forward(60.0)
                    .addTemporalMarker(0.7, ()->{
                        sisteme.lansat.setPower(LANSAT_POWER_AUTO-0.01);
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();
            Trajectory trajA2 = drive.trajectoryBuilder(trajA1.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(90.0))))
                    .lineToConstantHeading(new Vector2d(13.0, -55.0))
                    .build();*/
            Trajectory trajA1 = drive.trajectoryBuilder(startPose)
                    .forward(5.0)
                    .addTemporalMarker(0.5, ()->{ sisteme.cuva.setPosition(CUVA_SUS); })
                    .build();
            Trajectory trajA2 = drive.trajectoryBuilder(trajA1.end().plus(new Pose2d(0,0,Math.toRadians(8.5))))
                    .lineToLinearHeading(new Pose2d(13.0, -55.0, Math.toRadians(90.0)))
                    .addTemporalMarker(0.5, ()->{ sisteme.bratWobble.setPosition(BRAT_JOS); })
                    .addDisplacementMarker(()->{ sisteme.clawWobble.setPosition(CLAW_LASAT); })
                    .build();
            Trajectory trajA3 = drive.trajectoryBuilder(trajA2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-90.0))))
                    .lineToConstantHeading(new Vector2d(-34.0, -25.5))
                    .addTemporalMarker(0.5, ()->{ sisteme.bratWobble.setPosition(BRAT_JOS); })
                    .build();
            Trajectory trajA4 = drive.trajectoryBuilder(trajA3.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(90.0))))
                    .lineToConstantHeading(new Vector2d(22.0, -45.0))
                    .build();
            Trajectory trajA5 = drive.trajectoryBuilder(trajA4.end())
                    .forward(20.0)
                    .build();

            /*drive.followTrajectory(trajA1);

            drive.turn(Math.toRadians(-5.0));

            sleep(350); shoot(); sleep(350); shoot(); sleep(350); shoot();
            sisteme.lansat.setPower(0);

            drive.turn(Math.toRadians(95.0));
            sisteme.bratWobble.setPosition(BRAT_JOS);

            drive.followTrajectory(trajA2);
            sisteme.clawWobble.setPosition(CLAW_LASAT);*/

            sisteme.lansat.setVelocity(LANSAT_AUTO_PS);
            drive.followTrajectory(trajA1);
            sleep(1500);
            shoot();
            drive.turn(Math.toRadians(4.5));
            shoot();
            drive.turn(Math.toRadians(4.0));
            shoot();
            sisteme.lansat.setPower(0);

            drive.followTrajectory(trajA2);

            sisteme.bratWobble.setPosition(BRAT_SUS);
            drive.turn(Math.toRadians(-90.0));

            drive.followTrajectory(trajA3);

            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(1000);
            sisteme.clawWobble.setPosition(CLAW_PRINS);

            drive.turn(Math.toRadians(90.0));

            drive.followTrajectory(trajA4);

            sisteme.clawWobble.setPosition(CLAW_LASAT);
            sisteme.bratWobble.setPosition(BRAT_SUS);

            drive.followTrajectory(trajA5);

            sisteme.bratWobble.setPosition(BRAT_SUS);
            sisteme.cuva.setPosition(CUVA_JOS);

            PoseStorage.autoEndPose = drive.getPoseEstimate();

            while(opModeIsActive())
                ;

        } else if(ringsNumber == ONE) { // B
            /*Trajectory trajB1 = drive.trajectoryBuilder(startPose)
                    .forward(26.0)
                    //.splineToConstantHeading(new Vector2d(-32.0, -34.0), Math.toRadians(0.0))
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_B1);
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();
            Trajectory trajB12 = drive.trajectoryBuilder(trajB1.end())
                    .strafeLeft(8.0)
                    .build();
            Trajectory trajB2 = drive.trajectoryBuilder(trajB12.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-7.5))))
                    .forward(13.0)
                    .build();*/

            Trajectory trajB1 = drive.trajectoryBuilder(startPose)
                    .forward(5.0)
                    .addTemporalMarker(0.5, ()->{ sisteme.cuva.setPosition(CUVA_SUS); })
                    .build();
            Trajectory trajB2 = drive.trajectoryBuilder(trajB1.end().plus(new Pose2d(0,0,Math.toRadians(8.7))))
                    .forward(35.0)
                    .addTemporalMarker(0, ()->{
                        sisteme.cuva.setPosition(CUVA_JOS);
                        intakeOn();
                    })
                    .build();

            Trajectory trajB3 = drive.trajectoryBuilder(trajB2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-11.2))))
                    .lineToLinearHeading(new Pose2d(35.0, -44.0, Math.toRadians(180.0)))
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            Trajectory trajB4 = drive.trajectoryBuilder(trajB3.end())
                    .lineToLinearHeading(new Pose2d(-36.15, -19.0, Math.toRadians(0.0)))
                    .addTemporalMarker(0.3, ()->{sisteme.bratWobble.setPosition(BRAT_SUS);})
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            /*Trajectory trajB41 = drive.trajectoryBuilder(trajB4.end())
                    .strafeRight(5.0)
                    .build();*/
            Trajectory trajB5 = drive.trajectoryBuilder(trajB4.end())
                    .lineToLinearHeading(new Pose2d(20.0, -53.5, Math.toRadians(180.0)))
                    .build();
            Trajectory trajB6 = drive.trajectoryBuilder(trajB5.end())
                    .forward(10.0)
                    .build();

            /*drive.followTrajectory(trajB1);

            drive.followTrajectory(trajB12);

            drive.turn(Math.toRadians(-7.5));
            sleep(500);
            shoot(); sleep(800); shoot(); sleep(800); shoot();
            sisteme.lansat.setVelocity(LANSAT_AUTO_B1); // 0.065
            sisteme.cuva.setPosition(CUVA_JOS);
            intakeOn();

            sisteme.lansat.setVelocity(LANSAT_AUTO_B2);
            drive.followTrajectory(trajB2);*/

            sisteme.lansat.setVelocity(LANSAT_AUTO_PS);
            drive.followTrajectory(trajB1);
            sleep(1500);
            shoot();
            drive.turn(Math.toRadians(4.5));
            shoot();
            drive.turn(Math.toRadians(4.2));
            shoot();
            sisteme.lansat.setPower(0);

            drive.followTrajectory(trajB2);

            sisteme.lansat.setVelocity(LANSAT_AUTO_B2);
            sleep(800);
            intakeOff();
            sisteme.cuva.setPosition(CUVA_SUS); sleep(800);
            drive.turn(Math.toRadians(-11.2));
            shoot();
            sisteme.lansat.setPower(0);

            //drive.turn(Math.toRadians(7.5));

            drive.followTrajectory(trajB3);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajB4);


            sisteme.bratWobble.setPosition(BRAT_JOS);
            //drive.followTrajectory(trajB41);
            sleep(1000);
            sisteme.clawWobble.setPosition(CLAW_PRINS);

            drive.followTrajectory(trajB5);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajB6);

            PoseStorage.autoEndPose = drive.getPoseEstimate();

            sisteme.bratWobble.setPosition(BRAT_SUS);
            while(opModeIsActive())
                ;

        } else { // C
            /*sisteme.bratOprit.setPosition(BRAT_OPRIT_EXT_AUTO);
            sisteme.baraOprit.setPosition(BARA_OPRIT_EXT_AUTO);*/

            Trajectory trajC1 = myTrajectoryBuilder(startPose, 60, 60)
                    .splineToConstantHeading(new Vector2d(-28.0, -34.0), Math.toRadians(0.0))
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setPower(LANSAT_POWER_AUTO-0.03);
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();

            Trajectory trajC2 = myTrajectoryBuilder(trajC1.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-0.0))), 15, 10)
                    .forward(16)
                    .build();
            Trajectory trajC3 = drive.trajectoryBuilder(trajC2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(0.0))))
                    .forward(5.0)
                    .build();
            Trajectory trajC4 = myTrajectoryBuilder(trajC3.end(), 60, 60)
                    .lineToLinearHeading(new Pose2d(55.0, -65.0, Math.toRadians(180.0)))
                    .addTemporalMarker(0.5, ()->{intakeOff();})
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            Trajectory trajC5 = myTrajectoryBuilder(trajC4.end(), 60.0, 60.0)
                    .lineToConstantHeading(new Vector2d(-24.5, -23.0))
                    .build();
            Trajectory trajC51 = myTrajectoryBuilder(trajC5.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(183))), 60.0, 60.0)
                    .back(10.0)
                    .build();
            Trajectory trajC6 = myTrajectoryBuilder(trajC51.end(), 60.0, 60.0)
                    .lineToConstantHeading(new Vector2d(46.0, -65.0))
                    .build();
            Trajectory trajC7 = myTrajectoryBuilder(trajC6.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-180.0))), 60.0, 60.0)
                    .forward(30.0)
                    .build();


            drive.followTrajectory(trajC1);
            drive.turn(Math.toRadians(-7.0));
            shoot();sleep(200);shoot(); sleep(200); shoot();
            sisteme.cuva.setPosition(CUVA_JOS);
            intakeOn();
            sisteme.lansat.setPower(LANSAT_POWER_AUTO-0.1);

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
