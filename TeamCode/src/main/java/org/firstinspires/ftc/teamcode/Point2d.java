package org.firstinspires.ftc.teamcode;

import java.util.Locale;

class Point2d
{
    Point2d(String name, double x, double y)
    {
        this.name = name;
        this.x = x;
        this.y = y;
    }

    Point2d(double x, double y)
    {
        this(new String("PT" + numpts++), x, y);
    }

    double distance(Point2d tgtPt)
    {
        double sq_dist = (tgtPt.x - x)*(tgtPt.x -x) + (tgtPt.y - y)*(tgtPt.y - y);
        return Math.sqrt(sq_dist);
    }

    @SuppressWarnings("unused")
    double angle(Point2d prvPt, Point2d nxtPt)
    {
        double seg1FldHdg = Math.atan2(y - prvPt.getY(), x - prvPt.getX());
        double seg2FldHdg = Math.atan2(nxtPt.getY() - y, (nxtPt.getX() - x));
        return Math.toDegrees(seg2FldHdg - seg1FldHdg);
    }

    public String getName() { return name; };

    public double getX()
    {
        return x;
    }
    public double getY()
    {
        return y;
    }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }

    public String toString()
    {
        return String.format(Locale.US, "%s(%5.2f: %5.2f)",
                name, x, y);
    }

    private String name;
    private double x;
    private double y;
    private static int numpts = 0;
}
