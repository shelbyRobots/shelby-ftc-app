package org.firstinspires.ftc.teamcode;

public class Input_Shaper
{
    private boolean inputActive;
    private boolean inputSlow;

    public double shape(double inValue)
    {
        inputActive = (Math.abs(inValue) > 0.1);
        inputSlow   = (Math.abs(inValue) < 0.8);

        double sign = Math.signum(inValue);
        if(inputActive)
        {
            if(inputSlow)
            {
                double out = sign * (Math.abs(inValue)*4.0/7.0 - 0.4/7.0);
                return out;
            }
            else
            {
                double out = sign * (Math.abs(inValue)*3.0 - 2.0);
                return out;
            }
        }
        else
        {
            return 0.0f;
        }
    }
}
