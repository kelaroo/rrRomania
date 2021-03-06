package org.firstinspires.ftc.teamcode.autonome;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.systems.BaraOprit;
import org.firstinspires.ftc.teamcode.systems.Intake;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BARA_OPRIT_EXT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BRAT_JOS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BRAT_SUS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CLAW_LASAT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CUVA_JOS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CUVA_SUS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_BWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_FWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_LEFT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_RIGHT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_STATIONARY;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE3_LEFT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE3_SUCK;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_AUTO_A1;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_AUTO_B1;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
@Disabled
public class AutonomaRosuSuptTest extends LinearOpMode {
    CameraDemo camera;
    SampleMecanumDrive drive;
    RRHardwareConfig sisteme;

    private final CameraDemo.RingsDetectionPipeline.RingsNumber NONE = CameraDemo.RingsDetectionPipeline.RingsNumber.NONE;
    private final CameraDemo.RingsDetectionPipeline.RingsNumber ONE = CameraDemo.RingsDetectionPipeline.RingsNumber.ONE;
    private final CameraDemo.RingsDetectionPipeline.RingsNumber FOUR = CameraDemo.RingsDetectionPipeline.RingsNumber.FOUR;

    final Pose2d startPose = new Pose2d(-63.0, -40.0, Math.toRadians(0.0));

    final double LANSAT_REDS_C1 = 1460;
    final double LANSAT_REDS_C2 = 1420;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new CameraDemo(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        sisteme = new RRHardwareConfig(hardwareMap);

        drive.setPoseEstimate(startPose);

        CameraDemo.RingsDetectionPipeline.RingsNumber ringsNumber = CameraDemo.RingsDetectionPipeline.RingsNumber.FOUR;

        while(!isStarted()) {
            ringsNumber = CameraDemo.RingsDetectionPipeline.getNumberOfRings();
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
                    .forward(40.0)
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
                        intakeOn();
                    })
                    .build();
            Trajectory trajB2 = drive.trajectoryBuilder(trajB1.end())
                    .strafeLeft(8.0)
                    .build();
            Trajectory trajB21 = drive.trajectoryBuilder(trajB2.end())
                    .forward(24.0)
                    .build();

            // lasa wobble
            Trajectory trajB3 = drive.trajectoryBuilder(trajB21.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-11))))
                    .lineToLinearHeading(new Pose2d(34.0, -38.0, Math.toRadians(180.0)))
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();

            Trajectory trajB4 = drive.trajectoryBuilder(trajB3.end())
                    .forward(15.0)
                    .build();

            Trajectory trajB5 = drive.trajectoryBuilder(trajB4.end())
                    .strafeRight(28)
                    .build();

            drive.followTrajectory(trajB1);

            drive.followTrajectory(trajB2);

            //TODO: daca nu unghiu bun la lansat ( mai sus tre sa schimbi la traiectorie )
            drive.turn(Math.toRadians(-8));

            sleep(500);
            shoot(); sleep(500); shoot(); sleep(900); shoot();

            sisteme.lansat.setVelocity(LANSAT_AUTO_B1);
            sisteme.cuva.setPosition(CUVA_JOS);
            //intakeOn();

            drive.followTrajectory(trajB21);

            drive.turn(Math.toRadians(-7));
            sisteme.cuva.setPosition(CUVA_SUS); sleep(600);
            shoot(); sleep(100);
            sisteme.cuva.setPosition(CUVA_JOS);
            intakeOff();

            drive.followTrajectory(trajB3);

            sisteme.clawWobble.setPosition(CLAW_LASAT);

            drive.followTrajectory(trajB4);

            sisteme.lansat.setPower(0);

            sisteme.bratWobble.setPosition(BRAT_SUS);

            PoseStorage.autoEndPose = drive.getPoseEstimate();
            sisteme.lansat.setPower(0);

            while(opModeIsActive())
                ;

        } else { // C
            // merge sa lanseze
            Trajectory trajC1 = drive.trajectoryBuilder(startPose)
                    .forward(24)
                    .addTemporalMarker(0.3, ()->{
                        //sisteme.lansat.setVelocity(LANSAT_REDS_C1); // 0.023
                        sisteme.cuva.setPosition(CUVA_SUS);
                    })
                    .build();

            // se aliniaza sa suga
            Trajectory trajC12 = drive.trajectoryBuilder(trajC1.end())
                    .strafeLeft(10)
                    .build();

            // suge
            Trajectory trajC2 =  myTrajectoryBuilder(trajC12.end(), 7, 20)
                    .forward(15)
                    .build();

            // merge sa lase wobble
            Trajectory trajC4 = myTrajectoryBuilder(trajC2.end().plus(new Pose2d(0,0,Math.toRadians(-9.0))), 60, 60)
                    .lineToLinearHeading(new Pose2d(55.0, -52.0, Math.toRadians(180.0)))
                    .addTemporalMarker(0.7, ()->{sisteme.bratWobble.setPosition(BRAT_JOS);})
                    .build();

            Trajectory trajC41 = myTrajectoryBuilder(trajC4.end(), 60, 60)
                    .strafeRight(40)
                    .build();

            Trajectory trajC5 = myTrajectoryBuilder(trajC41.end(), 40, 40)
                    .forward(45.0)
                    .build();


            lowerBratOprit();
            drive.followTrajectory(trajC1);

            //TODO: aici e unghiu de lansat (tre sa schimbi mai sus unde e cu .plus(new Pose2d(....)))
            drive.turn(Math.toRadians(-6.0));
            shoot();sleep(70);shoot();sleep(70);shoot();

            //TODO: daca ai schimbat unghiu fix mai sus, schimba si aici
            drive.turn(Math.toRadians(6.0));

            drive.followTrajectory(trajC12);
            sisteme.cuva.setPosition(CUVA_JOS);

            intakeOn();
            drive.followTrajectory(trajC2); //sleep(250);
            intakeOut(); sleep(100);
            intakeOn(); sleep(300);
            intakeOff();
            sisteme.lansat.setPower(0);
            /*drive.followTrajectory(trajC21); sleep(250);
            sisteme.cuva.setPosition(CUVA_SUS);
            drive.followTrajectory(trajC212); sleep(250);

            drive.turn(Math.toRadians(-8.0));
            sleep(300);
            shoot();sleep(200);shoot();sleep(200);shoot();

            sisteme.cuva.setPosition(CUVA_JOS);

            drive.followTrajectory(trajC22); sleep(250);
            drive.followTrajectory(trajC23); sleep(250);
            sleep(500);
            intakeOff();

            sisteme.cuva.setPosition(CUVA_SUS); sleep(300);

            //TODO: aici e unghiul de la lansat 2
            //drive.turn(Math.toRadians(-9.0));
            shoot();sleep(200);shoot();sleep(200);shoot();

            sisteme.lansat.setPower(0);
            sisteme.cuva.setPosition(CUVA_JOS);

            drive.followTrajectory(trajC4);
            sisteme.clawWobble.setPosition(CLAW_LASAT);
            intakeOff();

            drive.followTrajectory(trajC41);

            drive.followTrajectory(trajC5);

            PoseStorage.autoEndPose = drive.getPoseEstimate();

            sisteme.bratWobble.setPosition(BRAT_JOS);*/

            while(opModeIsActive())
                continue;
        }
    }

    void wait(int millis) {
        ElapsedTime timer = new ElapsedTime();
        while(timer.milliseconds() < millis)
            drive.update();
    }

    void shoot() {
        sisteme.impins.setPosition(IMPINS_FWD); sleep(250);
        sisteme.impins.setPosition(IMPINS_BWD); sleep(250);
    }

    void intakeOn() {
        sisteme.intake2.setPosition(INTAKE2_RIGHT);
        sisteme.intake.setPower(1);
        sisteme.intake3.setPower(INTAKE3_SUCK);
    }

    void intakeOff() {
        sisteme.intake.setPower(0);
        sisteme.intake2.setPosition(INTAKE2_STATIONARY);
        sisteme.intake3.setPower(0);
    }
    void intakeOut() {
        sisteme.intake.setPower(-1);
        sisteme.intake3.setPower(-Intake.INTAKE3_IN);
        sisteme.intake2.setPosition(INTAKE2_LEFT);
    }

    void lowerBratOprit() {
        sisteme.bratOprit.setPosition(BaraOprit.BRAT_OPRIT_AUTO_C);
        sisteme.baraOprit.setPosition(BaraOprit.BARA_OPRIT_AUTO_C);
    }

    public TrajectoryBuilder myTrajectoryBuilder(Pose2d startPose, double maxVelo, double maxAccel){
        MinVelocityConstraint myVelConstraint = new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(maxAccel), new MecanumVelocityConstraint(maxVelo, DriveConstants.TRACK_WIDTH)));
        ProfileAccelerationConstraint myAccelConstraint = new ProfileAccelerationConstraint(maxAccel);
        return new TrajectoryBuilder(startPose, myVelConstraint, myAccelConstraint);
    }
}
