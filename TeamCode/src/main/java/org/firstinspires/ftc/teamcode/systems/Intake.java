package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake implements System {

    public static final double INTAKE_IN = 0.75;

    public static final double INTAKE2_IN = 1;
    public static final double INTAKE2_OUT = 0;
    public static final double INTAKE2_STATIONARY = 0.5;

    public static final double INTAKE3_IN = 0.75;

    public DcMotor intake;
    public Servo intake2;
    public DcMotor intake3;

    Robot robot;

    public enum IntakeState {
        OFF, SUGE, SCUIPA
    }
    public IntakeState intakeState = IntakeState.OFF;

    public Intake(HardwareMap hw, Robot r) {
        intake = hw.get(DcMotor.class, "intake");
        intake2 = hw.get(Servo.class, "intake2");
        intake3 = hw.get(DcMotor.class, "intake3");

        robot = r;
    }

    @Override
    public void update() {
        switch(intakeState) {
            case OFF:
                intake.setPower(0);
                intake2.setPosition(INTAKE2_STATIONARY);
                intake3.setPower(0);
                break;
            case SUGE:
                if(robot.cuva.cuvaState == Cuva.CuvaState.JOS) {
                    intake.setPower(INTAKE_IN);
                    intake2.setPosition(INTAKE2_IN);
                }
                intake3.setPower(INTAKE3_IN);
                break;
            case SCUIPA:
                if(robot.cuva.cuvaState == Cuva.CuvaState.JOS) {
                    intake2.setPosition(INTAKE2_OUT);
                    intake.setPower(-INTAKE_IN);
                }
                intake3.setPower(-INTAKE3_IN);
                break;
        }
    }
}