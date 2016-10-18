package org.firstinspires.ftc.teamcode;

public interface BeaconFinder
{
    LightOrder getLightOrder();
    double getLButtonPos();
    double getRButtonPos();

    enum LightOrder
    {
        UNKNOWN,
        RED_RED,
        RED_BLUE,
        BLUE_RED,
        BLUE_BLUE
    }
}
