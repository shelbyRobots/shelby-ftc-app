package org.firstinspires.ftc.teamcode;

public interface BeaconFinder
{
    LightOrder getLightOrder();

    public BeaconSide getRedPosSide();
    public BeaconSide getBluePosSide();

    public double getBeaconConf();
    public double getBeaconPosX();
    public double getBeaconPosZ();

    //double getLButtonPos();
    //double getRButtonPos();

    enum BeaconSide
    {
        UNKNOWN,
        LEFT,
        RIGHT
    }

    enum LightOrder
    {
        UNKNOWN,
        RED_RED,
        RED_BLUE,
        BLUE_RED,
        BLUE_BLUE
    }
}
