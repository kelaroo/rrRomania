package org.firstinspires.ftc.teamcode.tester;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import javax.crypto.spec.DHGenParameterSpec;

public class TooWaySpinningWheelConfig {
    DcMotor base;
    DcMotor wheel;

    DigitalChannel button;

    public void TooWaySpinningWheel(HardwareMap hw){
        base = hw.get(DcMotor.class, "base");
        wheel = hw.get(DcMotor.class, "wheel");

        button = hw.get(DigitalChannel.class, "button");
        button.setMode(DigitalChannel.Mode.INPUT);
    }
}
