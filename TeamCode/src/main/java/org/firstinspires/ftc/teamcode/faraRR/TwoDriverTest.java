package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.systems.BaraOprit;
import org.firstinspires.ftc.teamcode.systems.Cuva;
import org.firstinspires.ftc.teamcode.systems.Impins;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Lansat;
import org.firstinspires.ftc.teamcode.systems.Robot;
import org.firstinspires.ftc.teamcode.systems.Wobble;
import org.firstinspires.ftc.teamcode.util.GamepadEx;

@TeleOp
public class TwoDriverTest extends OpMode {

    Robot robot;
    GamepadEx gpad1;
    GamepadEx gpad2;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);

        gpad1 = new GamepadEx(gamepad1);
        gpad2 = new GamepadEx(gamepad2);

        gpad1.update();
        gpad2.update();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        // Trebuie pus la inceput ca sa ia input-urile
        gpad1.update();
        gpad2.update();

        //region Ultimata
        if(gpad1.dpad_left_once && gpad1.b) {
            robot.robotState = Robot.RobotState.MOVE_TO_PS;
        }
        if(robot.robotState != Robot.RobotState.MANUAL) {
            robot.update();
            return;
        }
        //endregion

        //region Drive
        if(gpad1.b)
            robot.robotSpeed = Robot.RobotSpeed.HIGH;
        else if(gpad1.x)
            robot.robotSpeed = Robot.RobotSpeed.LOW;

        double drive = -gpad1.left_stick_y;
        double strafe = gpad1.left_stick_x;
        double rotate = gpad1.right_stick_x;

        double RF = Range.clip(drive - strafe - rotate, -1, 1) * robot.robotSpeed.coeff;
        double RB = Range.clip(drive + strafe - rotate, -1, 1) * robot.robotSpeed.coeff;
        double LB = Range.clip(drive - strafe + rotate, -1, 1) * robot.robotSpeed.coeff;
        double LF = Range.clip(drive + strafe + rotate, -1, 1) * robot.robotSpeed.coeff;

        robot.drive.setMotorPowers(LF, LB, RB, RF);
        //endregion

        //region Intake
        if(gpad2.left_trigger > 0.2)
            robot.intake.intakeState = Intake.IntakeState.SUGE;
        else if(gpad2.right_trigger > 0.2)
            robot.intake.intakeState = Intake.IntakeState.SCUIPA;
        else
            robot.intake.intakeState = Intake.IntakeState.OFF;
        //endregion

        //region Cuva
        if(gpad2.y)
            robot.cuva.cuvaState = Cuva.CuvaState.SUS;
        else if(gpad2.a)
            robot.cuva.cuvaState = Cuva.CuvaState.JOS;
        //endregion

        //region Impins
        if(gpad2.left_bumper_once) {
            if (robot.cuva.cuvaState == Cuva.CuvaState.SUS)
                robot.impins.impinsState = Impins.ImpinsState.AUTO;
        }
        else if(robot.impins.impinsState == Impins.ImpinsState.MANUAL) {
            if(robot.cuva.cuvaState == Cuva.CuvaState.SUS) {
                if (gpad2.x)
                    robot.impins.impinsPosition = Impins.ImpinsPosition.FWD;
                else
                    robot.impins.impinsPosition = Impins.ImpinsPosition.BACK;
            }
            else
                robot.impins.impinsPosition = Impins.ImpinsPosition.SECOND;
        }
        //endregion

        //region Lansat
        if(gpad2.right_bumper){
            robot.lansat.lansatState = Lansat.LansatState.HIGH_GOAL;
        } else if(gpad2.b){
            robot.lansat.lansatState = Lansat.LansatState.POWERSHOTS;
        } else {
            robot.lansat.lansatState = Lansat.LansatState.IDLE;
        }
        //endregion

        //region Wobble
        if(gpad2.dpad_up_once)
            robot.wobble.bratState = Wobble.BratState.SUS;
        else if(gpad2.dpad_down_once)
            robot.wobble.bratState = Wobble.BratState.JOS;

        if(gpad2.dpad_left_once)
            robot.wobble.clawState = Wobble.ClawState.LASAT;
        else if(gpad2.dpad_right_once)
            robot.wobble.clawState = Wobble.ClawState.PRINS;
        //endregion

        //region BaraOprit
        if(gpad1.right_bumper)
            robot.baraOprit.opritState = BaraOprit.OpritState.INT;
        else if(gpad1.right_trigger > 0.2)
            robot.baraOprit.opritState = BaraOprit.OpritState.EXT;
        //endregion

        // !Trebuie pus la final ca sa ruleze sistemele
        robot.update();
    }
}