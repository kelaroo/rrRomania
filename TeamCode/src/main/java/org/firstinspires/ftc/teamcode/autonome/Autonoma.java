package org.firstinspires.ftc.teamcode.autonome;

import android.view.textclassifier.TextClassifierEvent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.*;

import org.apache.commons.math3.analysis.solvers.BracketedRealFieldUnivariateSolver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

import java.io.SequenceInputStream;
import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
@Disabled
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
            // merge spre lansat
            Trajectory trajA1 = drive.trajectoryBuilder(startPose)
                    .forward(60.0)
                    .addTemporalMarker(0.7, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_A1);
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();

            // merge sa lase wobble
            Trajectory trajA2 = drive.trajectoryBuilder(trajA1.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(90.0)))) //
                    .lineToConstantHeading(new Vector2d(13.0, -55.0))
                    .build();

            // da drumu la wobble
            Trajectory trajA21 = drive.trajectoryBuilder(trajA2.end())
                    .forward(5.0)
                    .build();

            // ia wobble 2
            Trajectory trajA3 = drive.trajectoryBuilder(trajA21.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-90.0))))
                    .lineToConstantHeading(new Vector2d(-41.0, -17.0))
                    .addTemporalMarker(0.5, ()->{ sisteme.bratWobble.setPosition(BRAT_JOS); })
                    .build();

            // merge sa lase wobble
            Trajectory trajA4 = drive.trajectoryBuilder(trajA3.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(90.0))))
                    .lineToConstantHeading(new Vector2d(22.0, -45.0))
                    .build();

            // parcheaza
            Trajectory trajA5 = drive.trajectoryBuilder(trajA4.end())
                    .forward(20.0)
                    .build();

            drive.followTrajectory(trajA1);

            drive.turn(Math.toRadians(-5.0));

            sleep(150); shoot(); sleep(150); shoot(); sleep(150); shoot();
            sisteme.lansat.setPower(0);

            drive.turn(Math.toRadians(95.0));
            sisteme.bratWobble.setPosition(BRAT_JOS);

            drive.followTrajectory(trajA2);
            sisteme.clawWobble.setPosition(CLAW_LASAT);
            drive.followTrajectory(trajA21);

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

            drive.turn(Math.toRadians(-90.0));

            PoseStorage.autoEndPose = drive.getPoseEstimate();

            while(opModeIsActive())
                ;

        } else if(ringsNumber == ONE) { // B
            // merge fata sa lanseze
            Trajectory trajB1 = drive.trajectoryBuilder(startPose)
                    .forward(26.0)
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_B1);
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();

            // se pune in fata inelelor
            Trajectory trajB12 = drive.trajectoryBuilder(trajB1.end())
                    .strafeLeft(8.0)
                    .build();

            // suge //TODO: aici e unghiu ----------------------------------------------------------------------------v
            Trajectory trajB2 = drive.trajectoryBuilder(trajB12.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-8.5))))
                    .forward(18.0)
                    .build();

            // lasa wobble
            Trajectory trajB3 = drive.trajectoryBuilder(trajB2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(0.0))))
                    .lineToLinearHeading(new Pose2d(34.0, -38.0, Math.toRadians(180.0)))
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();

            // merge sa ia wobble 2
            Trajectory trajB4 = drive.trajectoryBuilder(trajB3.end())
                    .lineToLinearHeading(new Pose2d(-39.0, -21.0, Math.toRadians(0.0)))
                    .addTemporalMarker(0.3, ()->{sisteme.bratWobble.setPosition(BRAT_SUS);})
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();

            // merge sa lase wobble
            Trajectory trajB5 = drive.trajectoryBuilder(trajB4.end())
                    .lineToLinearHeading(new Pose2d(23.0, -41.0, Math.toRadians(180.0)))
                    .build();

            // parcheaza
            Trajectory trajB6 = drive.trajectoryBuilder(trajB5.end())
                    .forward(10.0)
                    .build();

            drive.followTrajectory(trajB1);

            drive.followTrajectory(trajB12);

            //TODO: daca nu unghiu bun la lansat ( mai sus tre sa schimbi la traiectorie )
            drive.turn(Math.toRadians(-8.5));


            sleep(500);
            shoot(); sleep(500); shoot(); sleep(900); shoot();
            sisteme.lansat.setVelocity(LANSAT_AUTO_B1);
            sisteme.cuva.setPosition(CUVA_JOS);
            intakeOn();

            sisteme.lansat.setVelocity(LANSAT_AUTO_B2);
            drive.followTrajectory(trajB2);

            sisteme.lansat.setVelocity(LANSAT_AUTO_B2);
            sleep(800);
            intakeOff();
            sisteme.cuva.setPosition(CUVA_SUS); sleep(800);
            shoot();
            sisteme.lansat.setPower(0);


            drive.followTrajectory(trajB3);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajB4);


            sisteme.bratWobble.setPosition(BRAT_JOS);
            sleep(1000);
            sisteme.clawWobble.setPosition(CLAW_PRINS);

            drive.followTrajectory(trajB5);

            sisteme.clawWobble.setPosition(CLAW_LASAT);
            sisteme.bratWobble.setPosition(BRAT_SUS);

            drive.followTrajectory(trajB6);

            drive.turn(Math.toRadians(-180.0));

            PoseStorage.autoEndPose = drive.getPoseEstimate();

            while(opModeIsActive())
                ;

        } else { // C
            // merge sa lanseze
            Trajectory trajC1 = drive.trajectoryBuilder(startPose)
                    .forward(24.5)
                    .addTemporalMarker(0.3, ()->{
                        sisteme.lansat.setVelocity(LANSAT_AUTO_C1); // 0.023
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();

            // se aliniaza sa suga
            Trajectory trajC12 = drive.trajectoryBuilder(trajC1.end())
                    .strafeLeft(8.0)
                    .build();

            // suge
            Trajectory trajC2 = myTrajectoryBuilder(trajC12.end(), 15, 15)
                    .forward(4.5)
                    .addTemporalMarker(0, ()->{intakeOn();})
                    .build();

            // suge
            Trajectory trajC21 = myTrajectoryBuilder(trajC2.end(), 15, 15)
                    .forward(4.5)
                    .addDisplacementMarker(()->{ sisteme.lansat.setVelocity(LANSAT_AUTO_C2); })
                    .build();

            // suge dar e al treilea si nu mai face
            Trajectory trajC22 = myTrajectoryBuilder(trajC21.end(), 15, 15)
                    .forward(2.5)
                    .build();

            // merge sa lase wobble
            Trajectory trajC4 = myTrajectoryBuilder(trajC22.end().plus(new Pose2d(0,0,Math.toRadians(-9.0))), 60, 60)
                    .lineToLinearHeading(new Pose2d(55.0, -52.0, Math.toRadians(180.0)))
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();

            // merge sa ia wobble 2
            Trajectory trajC5 = myTrajectoryBuilder(trajC4.end(), 60.0, 60.0)
                    .lineToConstantHeading(new Vector2d(-32.0, -16.0))
                    .build();

            // merge putin ca sa ia wobble
            Trajectory trajC51 = myTrajectoryBuilder(trajC5.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(181))), 60.0, 60.0)
                    .back(5.0)
                    .build();

            // merge sa lase wobble
            Trajectory trajC6 = myTrajectoryBuilder(trajC51.end(), 60.0, 60.0)
                    .lineToConstantHeading(new Vector2d(48.5, -57.0))
                    .build();

            // merge sa parcheze
            Trajectory trajC7 = myTrajectoryBuilder(trajC6.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-183.0))), 60.0, 60.0)
                    .forward(30.0)
                    .addTemporalMarker(0, ()->{
                        sisteme.bratOprit.setPosition(BRAT_PARCAT);
                        sisteme.baraOprit.setPosition(BARA_PARCAT);})
                    .build();


            drive.followTrajectory(trajC1);

            //TODO: aici e unghiu de lansat (tre sa schimbi mai sus unde e cu .plus(new Pose2d(....)))
            drive.turn(Math.toRadians(-5.0));
            shoot();sleep(70);shoot();sleep(70);shoot();

            //TODO: daca ai schimbat unghiu fix mai sus, schimba si aici
            drive.turn(Math.toRadians(5.0));

            drive.followTrajectory(trajC12);
            sisteme.cuva.setPosition(CUVA_JOS);

            drive.followTrajectory(trajC2); sleep(250);
            drive.followTrajectory(trajC21); sleep(250);
            drive.followTrajectory(trajC22); sleep(250);
            intakeOff();



            sisteme.cuva.setPosition(CUVA_SUS);

            //TODO: aici e unghiul de la lansat 2
            drive.turn(Math.toRadians(-9.0));
            shoot();sleep(200);shoot();sleep(200);shoot();

            sisteme.lansat.setPower(0);
            sisteme.cuva.setPosition(CUVA_JOS);

            drive.followTrajectory(trajC4);
            sisteme.clawWobble.setPosition(CLAW_LASAT);
            intakeOff();

            drive.followTrajectory(trajC5);

            drive.turn(Math.toRadians(181));
            drive.followTrajectory(trajC51);



            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(350);
            sisteme.clawWobble.setPosition(CLAW_PRINS);

            drive.followTrajectory(trajC6);
            drive.turn(Math.toRadians(-183.0));

            sisteme.bratWobble.setPosition(BRAT_JOS); sleep(350);
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
