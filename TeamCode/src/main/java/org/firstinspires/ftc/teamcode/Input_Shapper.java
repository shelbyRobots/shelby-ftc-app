package org.firstinspires.ftc.teamcode;

/**
 * Created by Martin on 10/26/2016.
 */

public class Input_Shapper {

    float motor_speed = 0.0f;
    private boolean Joystick_Input_Unactive;
    private boolean Joystick_Inoput_Slow;

    public float Input(float joystick_value) {

        Joystick_Input_Unactive = (Math.abs(joystick_value) > 0.1);
        Joystick_Inoput_Slow    = (Math.abs(joystick_value) < 0.6f);

        if(Joystick_Input_Unactive) {
            if(Joystick_Inoput_Slow) {
                return joystick_value * 0.5f;
            }
            else{
                return joystick_value;
            }
        }
        else {
            return 0.0f;
        }
    }
}
