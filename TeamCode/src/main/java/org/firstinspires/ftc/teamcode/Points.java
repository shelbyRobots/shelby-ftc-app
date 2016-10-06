package org.firstinspires.ftc.teamcode;

import java.util.Vector;

class Points
{
    private final Vector<Point2d> initRedPoints()
    {
        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);
        points.add(new Point2d(STARTX,  STARTY));
        points.add(new Point2d(STARTX,  TGT_1_Y));
        points.add(new Point2d(TOUCH_X, TGT_1_Y));
        points.add(new Point2d(CLEAR_X, TGT_1_Y));
        points.add(new Point2d(CLEAR_X, TGT_2_Y));
        points.add(new Point2d(TOUCH_X, TGT_2_Y));
        points.add(new Point2d(CLEAR_X, TGT_2_Y));
        points.add(new Point2d(END_X,   END_Y));

        return points;
    }

    Points()
    {
        Vector<Point2d> redPoints  = initRedPoints();
        Vector<Point2d> bluePoints = initBluePoints(redPoints);

        revSegs = new Vector<>(MAX_SEGMENTS);
        revSegs.add("DRV2");
        revSegs.add("DRV5");

        redSegs  = initSegments(redPoints);
        blueSegs = initSegments(bluePoints);

        redTurns  = initTurns(redSegs);
        blueTurns = initTurns(blueSegs);
    }

    final Segment[] getSegments(Field.Alliance color)
    {
        Segment[] segments = redSegs;
        if (color == Field.Alliance.BLUE)
        {
            segments = blueSegs;
        }
        return segments;
    }

    final double[] getTurns(Field.Alliance color)
    {
        double[] turns = redTurns;
        if (color == Field.Alliance.BLUE)
        {
            turns = blueTurns;
        }
        return turns;
    }

    private Vector<Point2d> initBluePoints(Vector<Point2d> rpts)
    {
        Vector<Point2d> bpts = new Vector<>(rpts.size());

        for(Point2d rpt : rpts)
        {
            bpts.add(convertRtoB(rpt));
        }
        return bpts;
    }

    private Segment[] initSegments(Vector<Point2d> pts)
    {
        int numSegs = pts.size() - 1;
        Segment[] pathSegs = new Segment[numSegs];
        for(int s = 0; s < numSegs; ++s)
        {
            pathSegs[s] = new Segment("DRV" + s, pts.get(s), pts.get(s+1));
        }

        for(String r : revSegs)
        {
            Segment s = getSegment(r, pathSegs);
            if(s != null)
            {
                s.setDir(Segment.SegDir.REVERSE);
            }
        }

        return pathSegs;
    }

    private Segment getSegment(String name, Segment[] segs)
    {
        for (Segment pathSeg : segs)
        {
            String n = pathSeg.getName();
            if (n.equals(name)) return pathSeg;
        }
        return null;
    }

    private double[] initTurns(Segment[] segs)
    {
        int numTrns = segs.length - 1;

        double[] turns = new double[numTrns];
        for(int t = 0; t < numTrns; ++t)
        {
            double h2 = segs[t+1].getFieldHeading();
            if(segs[t+1].getDir() == Segment.SegDir.REVERSE) h2+=180.0;
            double h1 = segs[t].getFieldHeading();
            if(segs[t].getDir()   == Segment.SegDir.REVERSE) h1+=180.0;

            double ang = h2 - h1;

            while(ang >   180.0) ang -= 360.0;
            while(ang <= -180.0) ang += 360.0;
            turns[t] = ang;
        }
        return turns;
    }

    private Point2d convertRtoB(Point2d rpt)
    {
        double bx = -rpt.getY();
        double by = -rpt.getX();
        return new Point2d(bx,by);
    }

    private final static double REAR_OFFSET = ShelbyBot.REAR_OFFSET;
    private final static double FRNT_OFFSET = ShelbyBot.FRNT_OFFSET;

    private static final double S_WALL = Field.S_WALL_Y;
    private static final double W_WALL = Field.W_WALL_X;

    private static final double STARTX = -2 * 12;
    private static final double STARTY = S_WALL + REAR_OFFSET;
    private static final double TGT_1_Y = -1*12;
    private static final double TGT_2_Y =  3*12;
    private static final double END_X = -9.0;
    private static final double END_Y = -9.0;

    private static final double SAFETY = 3;
    private static final double CLEAR_X = W_WALL + FRNT_OFFSET + 6;
    private static final double TOUCH_X = W_WALL + FRNT_OFFSET + SAFETY;

    private final static int    MAX_SEGMENTS = 16;
    private final Vector<String>  revSegs;

    private Segment[] redSegs;
    private Segment[] blueSegs;

    private double[] redTurns;
    private double[] blueTurns;

    public static void main(String[] args)
    {
        Points ps = new Points();
        Segment[] rsegs = ps.getSegments(Field.Alliance.RED);
        Segment[] bsegs = ps.getSegments(Field.Alliance.BLUE);
        double[]  rtrns = ps.getTurns(Field.Alliance.RED);
        double[]  btrns = ps.getTurns(Field.Alliance.BLUE);

        for(int s = 0 ; s < rsegs.length; ++s)
        {
            System.out.println("Rseg " + s + " " + rsegs[s]);
            if(s < rsegs.length - 1) System.out.println("Rtrn " + s + " " + rtrns[s]);
        }

        for(int s = 0 ; s < bsegs.length; ++s)
        {
            System.out.println("Rseg " + s + " " + bsegs[s]);
            if(s < bsegs.length - 1) System.out.println("Btrn " + s + " " + btrns[s]);
        }
    }
}
