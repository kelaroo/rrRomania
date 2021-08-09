package org.firstinspires.ftc.teamcode.faraRR;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonome.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Impins;
//import org.firstinspires.ftc.teamcode.systems.threads.OneShotOnly;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.*;

@Deprecated
@Disabled
@TeleOp
public class TwoDriver extends OpMode {

    HardwareConfig hw;
    SampleMecanumDrive drive;

    Impins impins;

    Pose2d lastPose;

    Double[] lansatHistory = new Double[]{-1.0, -1.0, -1.0};

    double coeff = COEFF_SPEED_HIGH;

    enum CuvaState {
        SUS, JOS
    }
    CuvaState cuvaState = CuvaState.SUS;

    enum ShootState {
        SHOOTING, IDLE
    }
    volatile ShootState shootState = ShootState.IDLE;

    enum LansatState {
        IDLE, SHOOTING, POWER_SHOT
    }
    LansatState lansatState = LansatState.IDLE;

    enum RobotState {
        DRIVER, POWERSHOTS, MOVE_TO
    }
    RobotState robotState = RobotState.DRIVER;
    Thread tAutoPS = null;

    enum BaraState {
        INT, EXT
    }
    volatile BaraState baraState = BaraState.INT;
    Thread tMaturice;
    ElapsedTime baraTimer;

    @Override
    public void init() {
        hw = new HardwareConfig(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        /*impins.createInstance(hardwareMap);
        impins = Impins.getInstance();
        impins.impingeSecond();*/

        if(PoseStorage.autoEndPose == null)
            telemetry.addData("Start pose", "null");
        else {
            drive.setPoseEstimate(PoseStorage.autoEndPose);
            telemetry.addData("Start pose", "initialized");
        }

        baraTimer = new ElapsedTime();
    }

    @Override
    public void start() {
        super.start();
        hw.bratOprit.setPosition(BRAT_OPRIT_EXT);
        hw.baraOprit.setPosition(BARA_OPRIT_EXT);

        hw.cuva.setPosition(CUVA_JOS);
        cuvaState = CuvaState.JOS;
    }

    @Override
    public void loop() {
        telemetry.addData("Lansat speed", hw.lansat.getVelocity());
        for(int i = 0 ; i < 3; i++)
            telemetry.addData(String.format("Lansat %d", i), lansatHistory[i]);

        drive.update();

        if(robotState == RobotState.DRIVER) {
            if(gamepad1.dpad_up && gamepad1.y) {
                robotState = RobotState.POWERSHOTS;
                tAutoPS = new Thread(new AutoPowerShots());
                tAutoPS.start();
                return;
            } else if(gamepad1.dpad_left && gamepad1.b) {
                robotState = RobotState.MOVE_TO;
                Thread tMovePS = new Thread(new MoveToPowerShot());
                tMovePS.start();
                return;
            }
        } else
            return;

        /// Driver 1
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x * COEFF_ROTATE;

        if(gamepad1.b)
            coeff = COEFF_SPEED_HIGH;
        if(gamepad1.x)
            coeff = COEFF_SPEED_LOW;

        // Brate
        if(gamepad1.left_trigger > 0.2)
            hw.baraS.setPosition(BARAS_EXT);
        else if(gamepad1.left_bumper)
            hw.baraS.setPosition(BARAS_INT);

        // Bara oprit
        if(gamepad1.right_trigger > 0.2) {
            baraState = BaraState.EXT;
            hw.bratOprit.setPosition(BRAT_OPRIT_EXT);
        } else if(gamepad1.right_bumper) {
            baraState = BaraState.INT;
            hw.bratOprit.setPosition(BRAT_OPRIT_INT);
        }

        switch (baraState) {
            case INT:
                hw.baraOprit.setPosition(BARA_OPRIT_INT);
                baraTimer.reset();
                break;
            case EXT:
                if(baraTimer.milliseconds() < 100)
                    hw.baraOprit.setPosition(BARA_OPRIT_EXT);
                else if(baraTimer.milliseconds() < 200)
                    hw.baraOprit.setPosition(BARA_OPRIT_EXT2);
                else if(baraTimer.milliseconds() < 300)
                    baraTimer.reset();
        }

        double RF = hw.clipPower(drive - strafe - rotate) * coeff;
        double RB = hw.clipPower(drive + strafe - rotate) * coeff;
        double LB = hw.clipPower(drive - strafe + rotate) * coeff;
        double LF = hw.clipPower(drive + strafe + rotate) * coeff;

        hw.leftFront.setPower(LF);
        hw.leftBack.setPower(LB);
        hw.rightFront.setPower(RF);
        hw.rightBack.setPower(RB);

        /// Driver 2

        // Intake
        if(cuvaState == CuvaState.JOS) {
            if(gamepad2.left_trigger > 0.2) {
                hw.intake.setPower(INTAKE_SUCK);
                hw.intake2.setPosition(INTAKE2_RIGHT);
                hw.intake3.setPower(INTAKE3_SUCK);
            } else if(gamepad2.right_trigger > 0.2) {
                hw.intake.setPower(-INTAKE_SUCK);
                hw.intake2.setPosition(INTAKE2_LEFT);
                hw.intake3.setPower(-INTAKE3_SUCK);
            } else {
                hw.intake.setPower(0);
                hw.intake2.setPosition(INTAKE2_STATIONARY);
                hw.intake3.setPower(0);
            }
        } else {
            if(gamepad2.left_trigger > 0.2) {
                hw.intake3.setPower(INTAKE3_SUCK);
            } else if(gamepad2.right_trigger > 0.2) {
                hw.intake3.setPower(-INTAKE3_SUCK);
            } else {
                hw.intake.setPower(0);
                hw.intake2.setPosition(INTAKE2_STATIONARY);
                hw.intake3.setPower(0);
            }
        }

        // Cuva
        if(gamepad2.y) {
            hw.cuva.setPosition(CUVA_SUS);
            cuvaState = CuvaState.SUS;
        } else if(gamepad2.a) {
            hw.cuva.setPosition(CUVA_JOS);
            cuvaState = CuvaState.JOS;
        }

        // Impins
        /*if(gamepad2.left_bumper && cuvaState == CuvaState.SUS && impins.state == Impins.State.IDLE) {
            Thread tOneShot = new Thread(new OneShotOnly());
            tOneShot.start();
        } else if(impins.state == Impins.State.IDLE && cuvaState == CuvaState.SUS) {
            if(gamepad2.x)
                impins.impingeFwd();
            else
                impins.impingeBwd();
        } else if(impins.state == Impins.State.IDLE){
            impins.impingeSecond();
        }*/

        // Lansat
        // SWTICH VITEZA POWERSHOT BY CEI 2 TRAPPERI
        if(gamepad2.right_bumper)
            lansatState = LansatState.SHOOTING;
        else if(gamepad2.b)
            lansatState = LansatState.POWER_SHOT;
        else
            lansatState = LansatState.IDLE;

        if(lansatState == LansatState.SHOOTING)
            //hw.lansat.setPower(LANSAT_POWER);
            hw.lansat.setVelocity(LANSAT_SPEED);
        else if(lansatState == LansatState.POWER_SHOT)
            hw.lansat.setVelocity(LANSAT_SPEED_PS);
        else
            hw.lansat.setPower(0);

        // Wobble Arm
        if(gamepad2.dpad_up) {
            hw.bratWobble.setPosition(BRAT_SUS);
        } else if(gamepad2.dpad_down) {
            hw.bratWobble.setPosition(BRAT_JOS);
        }

        // Wobble claw
        if(gamepad2.dpad_right) {
            hw.clawWobble.setPosition(CLAW_PRINS);
        } else if(gamepad2.dpad_left && !gamepad2.dpad_up) {
            hw.clawWobble.setPosition(CLAW_LASAT);
        }
    }

    void waitTimer(int millis) {
        ElapsedTime timer = new ElapsedTime();
        while(timer.milliseconds() < millis)
            ;
    }

    void shoot() {
        ElapsedTime timer = new ElapsedTime();
        hw.impins.setPosition(IMPINS_FWD);
        timer.reset();
        while(timer.milliseconds() < 500)
            ;

        hw.impins.setPosition(IMPINS_BWD);
        timer.reset();
        while(timer.milliseconds() < 250)
            ;
    }

    // Imbeded classes
    private class OneButtonShoot implements Runnable {

        @Override
        public void run() {
            telemetry.addData("Thread", "started");
            ElapsedTime timer = new ElapsedTime();
            shootState = ShootState.SHOOTING;

            for(int i = 0; i < 3 && cuvaState == CuvaState.SUS; i++) {
                telemetry.addData("Thread", String.format("Shoot %d", i));
                hw.impins.setPosition(IMPINS_FWD);
                timer.reset();
                while(timer.milliseconds() < 250)
                    continue;

                lansatHistory[i] = hw.lansat.getVelocity();
                hw.impins.setPosition(IMPINS_BWD);
                timer.reset();
                while(timer.milliseconds() < 250)
                    continue;
            }

            shootState = ShootState.IDLE;
            //telemetry.update();
        }
    }

    private class AutoPowerShots implements Runnable {

        @Override
        public void run() {
            telemetry.addData("AutoPowerShots", "started");

            hw.cuva.setPosition(CUVA_SUS);
            cuvaState = CuvaState.SUS;

            robotState = RobotState.POWERSHOTS;

            // Porneste motor
            hw.lansat.setVelocity(LANSAT_SPEED_PS);
            waitTimer(3000);

            shoot();
            waitTimer(800);

            drive.turn(Math.toRadians(6.0));
            shoot();
            waitTimer(800);

            drive.turn(Math.toRadians(-8.3));
            shoot();

            hw.lansat.setPower(0);
            telemetry.update();
            robotState = RobotState.DRIVER;
        }
    }

    private class MoveToPowerShot implements Runnable {

        @Override
        public void run() {
            robotState = RobotState.MOVE_TO;

            hw.intake.setPower(INTAKE_SUCK);
            hw.intake2.setPosition(INTAKE2_RIGHT);
            hw.intake3.setPower(INTAKE3_SUCK);
            hw.lansat.setVelocity(LANSAT_SPEED_PS);
            /*hw.intake.setPower(0);
            hw.intake2.setPosition(INTAKE2_STATIONARY);
            hw.intake3.setPosition(INTAKE3_STATIONARY);*/
            waitTimer(100);
            hw.cuva.setPosition(CUVA_SUS);
            hw.intake2.setPosition(INTAKE2_STATIONARY);
            hw.intake3.setPower(0);
            waitTimer(1000);

            shoot();

            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(12.5) // 11 in
                    .addDisplacementMarker(()->{shoot();})
                    .build();
            drive.followTrajectory(traj);

            traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(11.5) // 11 in
                    .addDisplacementMarker(()->{shoot();})
                    .build();
            drive.followTrajectory(traj);

            robotState = RobotState.DRIVER;
        }
    }

    private class MaturiceUpDown implements Runnable {

        ElapsedTime timer;

        @Override
        public void run() {
            timer = new ElapsedTime();

            while(true) {
                switch (baraState) {
                    case INT:
                        hw.baraOprit.setPosition(BARA_OPRIT_INT);
                        timer.reset();
                        break;
                    case EXT:
                        if(timer.milliseconds() < 100)
                            hw.baraOprit.setPosition(BARA_OPRIT_EXT);
                        else if(timer.milliseconds() < 200)
                            hw.baraOprit.setPosition(BARA_OPRIT_EXT2);
                        else if(timer.milliseconds() < 300)
                            timer.reset();
                }
            }
        }
    }
}