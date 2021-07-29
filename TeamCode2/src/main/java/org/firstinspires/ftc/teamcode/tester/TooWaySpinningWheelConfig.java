package org.firstinspires.ftc.teamcode.tester;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import javax.crypto.spec.DHGenParameterSpec;

public class TooWaySpinningWheelConfig {
    DcMotor wheel;

    DigitalChannel button;

    public void TooWaySpinningWheel(HardwareMap hw){
        wheel = hw.get(DcMotor.class, "roata");

        button = hw.get(DigitalChannel.class, "button");
        button.setMode(DigitalChannel.Mode.INPUT);
    }
}
