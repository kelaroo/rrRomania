package org.firstinspires.ftc.teamcode.autonome;

import android.view.textclassifier.TextClassifierEvent;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig2.*;

import org.apache.commons.math3.analysis.solvers.BracketedRealFieldUnivariateSolver;
import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
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
            ringsNumber = Camera.RingsDetectionPipeline.getNumberOfRings();;
            telemetry.addData("ringsNumber", ringsNumber);
            telemetry.addData("nrPixels", Camera.RingsDetectionPipeline.nrPixels);
            telemetry.update();
        }

        sisteme.bratOprit.setPosition(BRAT_OPRIT_SUS);

        if(ringsNumber == NONE) { // A
            /*Trajectory trajA1 = drive.trajectoryBuilder(startPose)
                    .forward(60.0)
                    .addTemporalMarker(0.7, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_A1);
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();
            Trajectory trajA2 = drive.trajectoryBuilder(trajA1.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(90.0))))
                    .lineToConstantHeading(new Vector2d(13.0, -55.0))
                    .build();
            Trajectory trajA3 = drive.trajectoryBuilder(trajA2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-90.0))))
                    .lineToConstantHeading(new Vector2d(-36, -28.0))
                    //.addTemporalMarker(0.7, ()->{ sisteme.bratWobble.setPosition(BRAT_JOS); })
                    .build();
            Trajectory trajA4 = drive.trajectoryBuilder(trajA3.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(90.0))))
                    .lineToConstantHeading(new Vector2d(22.0, -45.0))
                    .build();
            Trajectory trajA5 = drive.trajectoryBuilder(trajA4.end())
                    .forward(20.0)
                    .build();

            drive.followTrajectory(trajA1);

            drive.turn(Math.toRadians(-6.0));

            sleep(350); shoot(); sleep(200); shoot(); sleep(200); shoot();
            sisteme.lansat.setPower(0);
            sisteme.impins.setPosition(IMPINS_SECOND);
            sisteme.cuva.setPosition(CUVA_JOS);

            drive.turn(Math.toRadians(96));
            sisteme.bratWobble.setPosition(BRAT_JOS);

            drive.followTrajectory(trajA2);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            sisteme.bratWobble.setPosition(BRAT_SUS);
            drive.turn(Math.toRadians(-90.0));

            drive.followTrajectory(trajA3);

            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(800);
            sisteme.clawWobble.setPosition(CLAW_PRINS); sleep(500);

            drive.turn(Math.toRadians(90.0));

            drive.followTrajectory(trajA4);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajA5);

            sisteme.bratWobble.setPosition(BRAT_SUS);
            sisteme.impins.setPosition(IMPINS_SECOND);
            sisteme.cuva.setPosition(CUVA_JOS);

            drive.turn(Math.toRadians(-90.0));*/

            Trajectory trajA1 = drive.trajectoryBuilder(startPose)
                    .splineToConstantHeading(new Vector2d(-20.0, -31.0), Math.toRadians(90.0))
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_A_PS); // 0.023
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();
            Trajectory trajA2 = drive.trajectoryBuilder(trajA1.end())
                    .strafeLeft(9)
                    .build();
            Trajectory trajA3 = drive.trajectoryBuilder(trajA2.end())
                    .strafeLeft(9)
                    .build();
            Trajectory trajA4 = drive.trajectoryBuilder(trajA3.end())
                    .lineToLinearHeading(new Pose2d(15, -50, Math.toRadians(90.0)))
                    .addTemporalMarker(0.5, ()->{ sisteme.bratWobble.setPosition(BRAT_JOS); })
                    .build();
            Trajectory trajA5 = drive.trajectoryBuilder(trajA4.end())
                    .lineToLinearHeading(new Pose2d(-41, -28, Math.toRadians(0.0)))
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            Trajectory trajA6 = drive.trajectoryBuilder(trajA5.end())
                    .lineToLinearHeading(new Pose2d(7, -50, Math.toRadians(90.0)))
                    .build();
            Trajectory trajA7 = drive.trajectoryBuilder(trajA6.end())
                    .forward(20.0)
                    .build();

            drive.followTrajectory(trajA1);
            shoot();

            drive.followTrajectory(trajA2);
            shoot();

            drive.followTrajectory(trajA3);
            shoot();

            sisteme.lansat.setPower(0);

            drive.followTrajectory(trajA4);
            sisteme.clawWobble.setPosition(CLAW_LASAT);
            sisteme.bratWobble.setPosition(BRAT_SUS);

            drive.followTrajectory(trajA5);
            sleep(500);
            sisteme.clawWobble.setPosition(CLAW_PRINS); sleep(100);

            drive.followTrajectory(trajA6);
            sisteme.clawWobble.setPosition(CLAW_LASAT);
            sisteme.bratWobble.setPosition(BRAT_SUS);

            drive.followTrajectory(trajA7);

            drive.turn(Math.toRadians(-90));

            while(opModeIsActive())
                ;

        } else if(ringsNumber == ONE) { // B
            /*Trajectory trajB1 = drive.trajectoryBuilder(startPose)
                    .forward(29.0)
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_B1); // 0.023
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();
            Trajectory trajB12 = drive.trajectoryBuilder(trajB1.end())
                    .strafeLeft(6.5)
                    .build();
            Trajectory trajB2 = myTrajectoryBuilder(trajB12.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-9.0))), 15, 15)
                    .forward(30.0)
                    .build();
            Trajectory trajB3 = drive.trajectoryBuilder(trajB2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(0))))
                    .lineToLinearHeading(new Pose2d(30.0, -43.0, Math.toRadians(180.0)))
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            Trajectory trajB4 = drive.trajectoryBuilder(trajB3.end())
                    .lineToLinearHeading(new Pose2d(-37.0, -27.7, Math.toRadians(0.0)))
                    .addTemporalMarker(0.1, ()->{sisteme.bratWobble.setPosition(BRAT_SUS);})
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            Trajectory trajB5 = drive.trajectoryBuilder(trajB4.end())
                    .lineToLinearHeading(new Pose2d(16.0, -46.5, Math.toRadians(180.0)))
                    .build();
            Trajectory trajB6 = drive.trajectoryBuilder(trajB5.end())
                    .forward(10.0)
                    .build();

            drive.followTrajectory(trajB1);

            drive.followTrajectory(trajB12);

            drive.turn(Math.toRadians(-9.0));
            sleep(200);
            shoot(); sleep(400); shoot(); sleep(400); shoot();
            sisteme.cuva.setPosition(CUVA_JOS);
            intakeOnB();

            drive.followTrajectory(trajB2);

            intakeOff();
            sisteme.lansat.setVelocity(LANSAT_AUTO_B2);
            sisteme.cuva.setPosition(CUVA_SUS); sleep(800);
            shoot();
            sisteme.lansat.setPower(0);


            drive.followTrajectory(trajB3);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajB4);

            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(1000);
            sisteme.clawWobble.setPosition(CLAW_PRINS); sleep(500);

            drive.followTrajectory(trajB5);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajB6);

            sisteme.bratWobble.setPosition(BRAT_SUS);

            drive.turn(Math.toRadians(-180.0));*/

            /*Trajectory trajB1 = drive.trajectoryBuilder(startPose)
                    .splineToConstantHeading(new Vector2d(-30, -55), Math.toRadians(0.0))
                    .splineToConstantHeading(new Vector2d(-20, -27), Math.toRadians(0.0))
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_B_PS); // 0.023
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();*/
            Trajectory trajB1 = drive.trajectoryBuilder(startPose)
                    .splineToConstantHeading(new Vector2d(-20, -50), 0.0)
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_B_PS); // 0.023
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();
            Trajectory trajB12 = drive.trajectoryBuilder(trajB1.end())
                    .strafeLeft(20)
                    .build();
            Trajectory trajB13 = drive.trajectoryBuilder(trajB12.end())
                    .strafeLeft(7.5)
                    .build();
            Trajectory trajB14 = drive.trajectoryBuilder(trajB13.end())
                    .strafeLeft(7.5)
                    .build();
            Trajectory trajB2 = drive.trajectoryBuilder(trajB14.end().plus(new Pose2d(0,0, Math.toRadians(-0))))
                    .lineToLinearHeading(new Pose2d(30.0, -43.0, Math.toRadians(180.0)))
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            Trajectory trajB3 = drive.trajectoryBuilder(trajB2.end())
                    .lineToLinearHeading(new Pose2d(-37, -28.0, Math.toRadians(0.0)))
                    .addTemporalMarker(0.1, ()->{sisteme.bratWobble.setPosition(BRAT_SUS);})
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            Trajectory trajB4 = drive.trajectoryBuilder(trajB3.end())
                    .lineToLinearHeading(new Pose2d(16.0, -46.5, Math.toRadians(180.0)))
                    .build();
            Trajectory trajB5 = drive.trajectoryBuilder(trajB4.end())
                    .forward(10)
                    .build();

            drive.followTrajectory(trajB1);
            drive.followTrajectory(trajB12);
            shoot();

            drive.followTrajectory(trajB13);
            shoot();

            drive.followTrajectory(trajB14);
            shoot();

            sisteme.lansat.setPower(0);
            sisteme.cuva.setPosition(CUVA_JOS);

            drive.followTrajectory(trajB2);
            sisteme.clawWobble.setPosition(CLAW_LASAT);
            sisteme.bratWobble.setPosition(BRAT_SUS);

            drive.followTrajectory(trajB3);

            sleep(500);
            sisteme.clawWobble.setPosition(CLAW_PRINS);
            sleep(500);

            drive.followTrajectory(trajB4);
            sisteme.clawWobble.setPosition(CLAW_LASAT);
            sisteme.bratWobble.setPosition(BRAT_SUS);

            drive.followTrajectory(trajB5);

            while(opModeIsActive())
                ;

        } else { // C
            /*Trajectory trajC1 = myTrajectoryBuilder(startPose, 60, 60)
                    .forward(22.0)
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_C1); // 0.023
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();
            Trajectory trajC12 = drive.trajectoryBuilder(trajC1.end())
                    .back(7)
                    .build();

            Trajectory trajC2 = myTrajectoryBuilder(trajC12.end().plus(new Pose2d(0,0,Math.toRadians(13))), 7, 15)
                    .forward(6.0)
                    .addTemporalMarker(0, ()->{intakeOn();})
                    .build();
            Trajectory trajC21 = myTrajectoryBuilder(trajC2.end(), 30, 15)
                    .forward(6.0)
                    .addDisplacementMarker(()->{ sisteme.lansat.setVelocity(LANSAT_AUTO_B2); })
                    .build();
            Trajectory trajC22 = myTrajectoryBuilder(trajC21.end(), 30, 15)
                    .forward(6.0)
                    .build();
            Trajectory trajC4 = myTrajectoryBuilder(trajC22.end().plus(new Pose2d(0,0,Math.toRadians(-7.0))), 60, 60)
                    .lineToLinearHeading(new Pose2d(55.0, -52.0, Math.toRadians(180.0)))
                    //.addTemporalMarker(0.5, ()->{intakeOff();})
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            Trajectory trajC5 = myTrajectoryBuilder(trajC4.end(), 60.0, 60.0)
                    .lineToConstantHeading(new Vector2d(-20.0, -30.0))
                    .build();
            Trajectory trajC51 = myTrajectoryBuilder(trajC5.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(178))), 60.0, 60.0)
                    .back(7)
                    .build();
            Trajectory trajC6 = myTrajectoryBuilder(trajC51.end(), 60.0, 60.0)
                    .lineToConstantHeading(new Vector2d(46.0, -52.0))
                    .build();
            Trajectory trajC7 = myTrajectoryBuilder(trajC6.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-180.0))), 60.0, 60.0)
                    .forward(30.0)
                    .build();


            *//*sisteme.baraOprit.setPosition(BARA_OPRIT_SUCK);*//*

            drive.followTrajectory(trajC1);
            drive.turn(Math.toRadians(-7));
            shoot();sleep(70);shoot();sleep(70);shoot();

            drive.turn(Math.toRadians(20));

            drive.followTrajectory(trajC12);
            intakeOn();
            sisteme.cuva.setPosition(CUVA_JOS);
            sisteme.bratOprit.setPosition(BRAT_OPRIT_C);

            drive.followTrajectory(trajC2); sleep(500);
            drive.followTrajectory(trajC21); sleep(500);
            drive.followTrajectory(trajC22); sleep(800);


            sisteme.cuva.setPosition(CUVA_SUS);
            drive.turn(Math.toRadians(-7.0));
            shoot();shoot();sleep(70);shoot();

            sisteme.lansat.setPower(0);
            sisteme.cuva.setPosition(CUVA_JOS);

            drive.followTrajectory(trajC4);
            sisteme.clawWobble.setPosition(CLAW_LASAT);
            intakeOff();

            drive.followTrajectory(trajC5);

            drive.turn(Math.toRadians(178));
            drive.followTrajectory(trajC51);

            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(350);
            sisteme.clawWobble.setPosition(CLAW_PRINS);

            drive.followTrajectory(trajC6);
            drive.turn(Math.toRadians(-180.0));

            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(350);
            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajC7);

            sisteme.bratWobble.setPosition(BRAT_SUS);

            while(opModeIsActive())
                continue;*/

            /*Trajectory trajC1 = drive.trajectoryBuilder(startPose)
                    .splineToConstantHeading(new Vector2d(-30, -55), Math.toRadians(0.0))
                    .splineToConstantHeading(new Vector2d(-20, -27.0), Math.toRadians(0.0))
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_C_PS); // 0.023
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();
            *//*Trajectory trajC12 = myTrajectoryBuilder(trajC1.end(), 60, 60)
                    .forward(5)
                    .build();*//*
            Trajectory trajC13 = drive.trajectoryBuilder(trajC1.end())
                    .strafeLeft(9)
                    .build();
            Trajectory trajC14 = drive.trajectoryBuilder(trajC13.end())
                    .strafeLeft(9)
                    .build();*/
            Trajectory trajC1 = drive.trajectoryBuilder(startPose)
                    .splineToConstantHeading(new Vector2d(-20, -50), 0.0)
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_B_PS); // 0.023
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();
            Trajectory trajC12 = drive.trajectoryBuilder(trajC1.end())
                    .strafeLeft(18)
                    .build();
            Trajectory trajC13 = drive.trajectoryBuilder(trajC12.end())
                    .strafeLeft(7.5)
                    .build();
            Trajectory trajC14 = drive.trajectoryBuilder(trajC13.end())
                    .strafeLeft(7.5)
                    .build();
            Trajectory trajC2 = drive.trajectoryBuilder(trajC14.end())
                    .lineToLinearHeading(new Pose2d(50, -57, Math.toRadians(180.0)))
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            Trajectory trajC3 = drive.trajectoryBuilder(trajC2.end())
                    .lineToLinearHeading(new Pose2d(-38, -26.0, Math.toRadians(0.0)))
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();
            Trajectory trajC4 = drive.trajectoryBuilder(trajC3.end())
                    .lineToLinearHeading(new Pose2d(43, -60, Math.toRadians(180.0)))
                    .build();
            Trajectory trajC5 = drive.trajectoryBuilder(trajC4.end())
                    .forward(30.0)
                    .build();

            drive.followTrajectory(trajC1);
            drive.followTrajectory(trajC12);

            //drive.followTrajectory(trajC12);
            shoot();

            drive.followTrajectory(trajC13);
            shoot();

            drive.followTrajectory(trajC14);
            shoot();

            sisteme.lansat.setPower(0);
            sisteme.cuva.setPosition(CUVA_JOS);

            /*drive.turn(Math.toRadians(-7.0));
            shoot();sleep(70);shoot();sleep(70);shoot();*/

            drive.followTrajectory(trajC2);
            sisteme.clawWobble.setPosition(CLAW_LASAT);
            sisteme.bratWobble.setPosition(BRAT_SUS);

            drive.followTrajectory(trajC3);

            sleep(500);
            sisteme.clawWobble.setPosition(CLAW_PRINS);
            sleep(500);

            drive.followTrajectory(trajC4);
            sisteme.clawWobble.setPosition(CLAW_LASAT);
            sisteme.bratWobble.setPosition(BRAT_SUS);

            drive.followTrajectory(trajC5);

        }
    }

    void shoot() {
        sisteme.impins.setPosition(IMPINS_FWD); sleep(250);
        sisteme.impins.setPosition(IMPINS_BWD); sleep(250);
    }

    void intakeOn() {
        sisteme.intake.setPower(INTAKE_SUCK);
        sisteme.intake2.setPosition(INTAKE2_RIGHT);
        sisteme.intake3.setPower(INTAKE3_SUCK);
    }
    void intakeOnB() {
        sisteme.intake.setPower(INTAKE_AUTO_B);
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
