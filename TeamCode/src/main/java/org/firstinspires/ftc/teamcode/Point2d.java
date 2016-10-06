package org.firstinspires.ftc.teamcode;

import java.util.Locale;

class Point2d
{
    Point2d(double x, double y)
    {
        this.x = x;
        this.y = y;
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

    public double getX()
    {
        return x;
    }

    public double getY()
    {
        return y;
    }

    public String toString()
    {
        return String.format(Locale.US, "(%5.2f, %5.2f)", x, y);
    }

    private double x;
    private double y;
}
