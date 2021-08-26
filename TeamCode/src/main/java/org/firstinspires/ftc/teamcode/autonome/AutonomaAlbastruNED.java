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

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BRAT_JOS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BRAT_SUS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CLAW_LASAT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_BWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_FWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_RIGHT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_STATIONARY;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE3_SUCK;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomaAlbastruNED extends LinearOpMode {
    CameraDemoAlbastru camera;
    SampleMecanumDrive drive;
    RRHardwareConfig sisteme;

    private final CameraDemoAlbastru.RingsDetectionPipeline.RingsNumber NONE = CameraDemoAlbastru.RingsDetectionPipeline.RingsNumber.NONE;
    private final CameraDemoAlbastru.RingsDetectionPipeline.RingsNumber ONE = CameraDemoAlbastru.RingsDetectionPipeline.RingsNumber.ONE;
    private final CameraDemoAlbastru.RingsDetectionPipeline.RingsNumber FOUR = CameraDemoAlbastru.RingsDetectionPipeline.RingsNumber.FOUR;

    final Pose2d startPose = new Pose2d(-63.0, 40.0, Math.toRadians(0.0)); //81 cm // 28.5cm

    final double LANSAT_BLUE_A1 = 1420;

    final double LANSAT_REDS_B1 = 1430;
    final double LANSAT_REDS_B2 = 1380;

    final double LANSAT_BLUES_C1 = 1450;
    final double LANSAT_BLUES_C2 = 1400;
    final double LANSAT_BLUES_C3 = 1390;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new CameraDemoAlbastru(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        sisteme = new RRHardwareConfig(hardwareMap);

        drive.setPoseEstimate(startPose);

        CameraDemoAlbastru.RingsDetectionPipeline.RingsNumber ringsNumber = CameraDemoAlbastru.RingsDetectionPipeline.getNumberOfRings();

        while(!isStarted()) {
            ringsNumber = CameraDemoAlbastru.RingsDetectionPipeline.getNumberOfRings();
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

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .forward(10)
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-60, 6))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-5.0, 6))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(15)
                .build();

        drive.followTrajectory(traj0);
        drive.followTrajectory(traj1);

        sisteme.cuva.setPosition(Cuva.CUVA_SUS);
        sleep(15000);

        sisteme.lansat.setVelocity(LANSAT_BLUE_A1);

        drive.followTrajectory(traj2);

        drive.turn(Math.toRadians(15));

        sleep(150); shoot(); sleep(150); shoot(); sleep(150); shoot();

        drive.turn(Math.toRadians(-15));

        drive.followTrajectory(traj3);



            while(opModeIsActive())
                continue;

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
