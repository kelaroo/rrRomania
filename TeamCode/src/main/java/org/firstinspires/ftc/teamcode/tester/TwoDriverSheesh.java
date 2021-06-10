package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonome.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.faraRR.HardwareConfig;
import org.firstinspires.ftc.teamcode.faraRR.TwoDriver;

import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BARAS_EXT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BARAS_INT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BARA_OPRIT_EXT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BARA_OPRIT_INT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BRAT_JOS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BRAT_OPRIT_EXT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BRAT_OPRIT_INT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.BRAT_SUS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CLAW_LASAT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CLAW_PRINS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.COEFF_ROTATE;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.COEFF_SPEED_HIGH;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.COEFF_SPEED_LOW;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CUVA_JOS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.CUVA_SUS;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_BWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_FWD;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.IMPINS_SECOND;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_LEFT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_RIGHT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE2_STATIONARY;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE3_LEFT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE3_RIGHT;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE3_STATIONARY;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE3_SUCK;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.INTAKE_SUCK;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_SPEED;
import static org.firstinspires.ftc.teamcode.faraRR.PowersConfig.LANSAT_SPEED_PS;

@TeleOp
public class TwoDriverSheesh extends OpMode {

    HardwareConfig hw;
    SampleMecanumDrive drive;

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

    boolean sheeshFound = false;
    int sheeshSoundID;

    SoundManager soundManager;
    ElapsedTime oosCD = null;

    @Override
    public void init() {
        hw = new HardwareConfig(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        if(PoseStorage.autoEndPose == null)
            telemetry.addData("Start pose", "null");
        else {
            drive.setPoseEstimate(PoseStorage.autoEndPose);
            telemetry.addData("Start pose", "initialized");
        }

        //lastPose = drive.getPoseEstimate();

        soundManager = new SoundManager(hardwareMap);

        if(!soundManager.addFile("sheesh1"))
            telemetry.addData("sheesh1", "Not found (add sheesh1 to src/res/raw)");
        if(!soundManager.addFile("sheesh2"))
            telemetry.addData("sheesh3", "Not found (add sheesh2 to src/res/raw)");
        if(!soundManager.addFile("sheesh3"))
            telemetry.addData("sheesh3", "Not found (add sheesh3 to src/res/raw)");
        if(!soundManager.addFile("oos"))
            telemetry.addData("oos", "Not found (add oos to src/res/raw)");

        telemetry.addData("sheesh sound resource", sheeshFound? "Found": "Can't find\n Add sheesh.wav to /src/main/res/raw");
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
            hw.baraOprit.setPosition(BARA_OPRIT_EXT);
            hw.bratOprit.setPosition(BRAT_OPRIT_EXT);
        } else if(gamepad1.right_bumper) {
            hw.baraOprit.setPosition(BARA_OPRIT_INT);
            hw.bratOprit.setPosition(BRAT_OPRIT_INT);
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
        if(gamepad2.left_bumper && cuvaState == CuvaState.SUS && shootState == ShootState.IDLE) {
            /*Thread tAutoShoot = new Thread(new OneButtonShoot());
            tAutoShoot.start();*/
            Thread tOneShot = new Thread(new OneShotOnly());
            tOneShot.start();
        } else if(shootState == ShootState.IDLE && cuvaState == CuvaState.SUS) {
            if(gamepad2.x)
                hw.impins.setPosition(IMPINS_FWD);
            else
                hw.impins.setPosition(IMPINS_BWD);
        } else if(shootState == ShootState.IDLE){
            hw.impins.setPosition(IMPINS_SECOND);
        }

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
            if(oosCD == null || oosCD.milliseconds() >= 700) {
                soundManager.playSound("oos");
                oosCD = new ElapsedTime();
            }
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

    private class OneShotOnly implements Runnable {

        void shoot(double time) {
            ElapsedTime timer = new ElapsedTime();
            hw.impins.setPosition(IMPINS_FWD);
            while(timer.milliseconds() < time)
                ;
            hw.impins.setPosition(IMPINS_BWD);
            timer.reset();
            while(timer.milliseconds() < 200)
                ;

        }

        @Override
        public void run() {
            ElapsedTime timer = new ElapsedTime();
            shootState = ShootState.SHOOTING;

            soundManager.playSound("sheesh1");
            shoot(500);

            soundManager.playSound("sheesh2");
            shoot(400);

            soundManager.playSound("sheesh3");
            shoot(250);

            shootState = ShootState.IDLE;
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

            soundManager.playSound("sheesh1");
            shoot();

            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(11.0)
                    .addDisplacementMarker(()->{shoot();soundManager.playSound("sheesh2");})
                    .build();
            drive.followTrajectory(traj);

            traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(11.0)
                    .addDisplacementMarker(()->{shoot();soundManager.playSound("sheesh3");})
                    .build();
            drive.followTrajectory(traj);

            robotState = RobotState.DRIVER;
        }
    }
}