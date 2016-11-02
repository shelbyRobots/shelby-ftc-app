package org.firstinspires.ftc.teamcode;


import java.util.Locale;

class Segment
{
    Segment(String name, Point2d start, Point2d end)
    {
        this.name   = name;
        this.strtPt = start;
        this.tgtPt  = end;
        this.fldHdg = angle(tgtPt);
        len = start.distance(end);
    }

    SegDir getDir()
    {
        return dir;
    }

    double getFieldHeading()
    {
        return fldHdg;
    }

    String getName()
    {
        return name;
    }

    Point2d getStrtPt()
    {
        return strtPt;
    }

    Point2d getTgtPt()
    {
        return tgtPt;
    }

    double getLength() { return len; }

    void setDir(SegDir dir)
    {
        this.dir = dir;
    }

    double angle(Point2d tgtPt)
    {
        double tgtFldHdg = Math.atan2(tgtPt.getY() - strtPt.getY(), (tgtPt.getX() - strtPt.getX()));
        return Math.toDegrees(tgtFldHdg);
    }

    public String toString()
    {
        return String.format(Locale.US, "%s: %s - %s %s %5.2f",
                name, strtPt, tgtPt, dir, fldHdg);
    }

    enum SegDir {FORWARD, REVERSE}
    private double  fldHdg = 0.0;
    private Point2d strtPt;
    private Point2d tgtPt;
    private String  name;
    private SegDir dir = SegDir.FORWARD;
    private double len = 0;

    //pointer to action function (i.e. shoot ball, findAndPushButton)
}
