package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx {

    public boolean dpad_up, dpad_down, dpad_right, dpad_left;
    public boolean dpad_up_once, dpad_down_once, dpad_right_once, dpad_left_once;

    public boolean y, b, a, x;
    public boolean y_once, b_once, a_once, x_once;

    public boolean left_bumper, right_bumper;
    public boolean left_bumper_once, right_bumper_once;

    public boolean left_stick_button, right_stick_button;
    public boolean left_stick_button_once, right_stick_button_once;

    public double left_trigger, right_trigger;
    public double left_stick_x, left_stick_y, right_stick_x, right_stick_y;

    Gamepad gamepad;

    public GamepadEx(Gamepad g) {
        gamepad = g;
    }

    public void update() {
        // Triggers
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;

        // Joysticks
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;

        //region Dpad
        if(gamepad.dpad_up) {
            if(!dpad_up)
                dpad_up_once = true;
            else
                dpad_up_once = false;
            dpad_up = true;
        } else {
            dpad_up = false;
            dpad_up_once = false;
        }
        if(gamepad.dpad_down) {
            if(!dpad_down)
                dpad_down_once = true;
            else
                dpad_down_once = false;
            dpad_down = true;
        } else {
            dpad_down = false;
            dpad_down_once = false;
        }
        if(gamepad.dpad_right) {
            if(!dpad_right)
                dpad_right_once = true;
            else
                dpad_right_once = false;
            dpad_right = true;
        } else {
            dpad_right = false;
            dpad_right_once = false;
        }
        if(gamepad.dpad_left) {
            if(!dpad_left)
                dpad_left_once = true;
            else
                dpad_left_once = false;
            dpad_left = true;
        } else {
            dpad_left = false;
            dpad_left_once = false;
        }
        //endregion

        //region RightButtons
        if(gamepad.y) {
            if(!y)
                y_once = true;
            else
                y_once = false;
            y = true;
        } else {
            y = false;
            y_once = false;
        }
        if(gamepad.b) {
            if(!b)
                b_once = true;
            else
                b_once = false;
            b = true;
        } else {
            b = false;
            b_once = false;
        }
        if(gamepad.a) {
            if(!a)
                a_once = true;
            else
                a_once = false;
            a = true;
        } else {
            a = false;
            a_once = false;
        }
        if(gamepad.x) {
            if(!x)
                x_once = true;
            else
                x_once = false;
            x = true;
        } else {
            x = false;
            x_once = false;
        }
        //endregion

        //region Bumpers
        if(gamepad.left_bumper) {
            if(!left_bumper)
                left_bumper_once = true;
            else
                left_bumper_once = false;
            left_bumper = true;
        } else {
            left_bumper = false;
            left_bumper_once = false;
        }
        if(gamepad.right_bumper) {
            if(!right_bumper)
                right_bumper_once = true;
            else
                right_bumper_once = false;
            right_bumper = true;
        } else {
            right_bumper = false;
            right_bumper_once = false;
        }
        //endregion

        //region Joystick Buttons
        if(gamepad.left_stick_button) {
            if(!left_stick_button)
                left_stick_button_once = true;
            else
                left_stick_button_once = false;
            left_stick_button = true;
        } else {
            left_stick_button = false;
            left_stick_button_once = false;
        }
        if(gamepad.right_stick_button) {
            if(!right_stick_button)
                right_stick_button_once = true;
            else
                right_stick_button_once = false;
            right_stick_button = true;
        } else {
            right_stick_button = false;
            right_stick_button_once = false;
        }
        //endregion
    }
}
