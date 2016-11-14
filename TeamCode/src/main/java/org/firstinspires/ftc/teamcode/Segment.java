package org.firstinspires.ftc.teamcode;


import java.util.Locale;

class Segment
{
    Segment(String name, Point2d start, Point2d end)
    {
        this.name   = name;
        this.strtPt = start;
        this.tgtPt  = end;
        this.fldHdg = angle();
        len = start.distance(end);
        this.dir = SegDir.FORWARD;
        this.speed = DEF_SEG_SPD;
        this.act = Action.NOTHING;
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
    double getSpeed() { return speed; }
    Action getAction() { return act; }
    double getLength() { return len; }

    void setAction(Action act) { this.act = act; }
    void setDir(SegDir dir)
    {
        this.dir = dir;
    }
    void setSpeed(double spd) { this.speed = spd; }

    double angle()
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
    enum Action {NOTHING, SHOOT, SCAN_IMAGE, FIND_BEACON, PUSH, RST_PUSHER}
    private static final double DEF_SEG_SPD = 0.5;
    private double  fldHdg = 0.0;
    private Point2d strtPt;
    private Point2d tgtPt;
    private String  name;
    private SegDir dir = SegDir.FORWARD;
    private double speed;
    private double len = 0;
    private Action act = Action.NOTHING;

    public static void main(String[] args)
    {
        Point2d ptA = new Point2d(-12.0, -67.0);
        Point2d ptB = new Point2d(-12.0, -64.0);
        Segment seg = new Segment("SEG", ptA, ptB);
        System.out.print("FLDHDG:" + seg.getFieldHeading() + "\n");
    }
}
