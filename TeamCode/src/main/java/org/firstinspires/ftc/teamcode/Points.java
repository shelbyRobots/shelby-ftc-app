package org.firstinspires.ftc.teamcode;

import java.util.Locale;
import java.util.Vector;

class Points
{
    private Vector<Point2d> initPoints()
    {
        Point2d start_pt = START_PT;
        Point2d presh_pt = PRSHT_PT;
        Point2d shoot_pt = SHOOT_PT;
        Point2d aim_pt   = AIMTO_PT;
        if(autoStrategy == Field.AutoStrategy.ANGSHOOT_PARKCNTR      ||
           autoStrategy == Field.AutoStrategy.ANGSHOOT_PARKCRNR      ||
           autoStrategy == Field.AutoStrategy.ANGSHOOT_PUSH_PARKCNTR ||
           autoStrategy == Field.AutoStrategy.ANGSHOOT_PUSH_PARKCRNR)
        {
            start_pt = ASTART_PT;
            presh_pt = APRSHT_PT;
            shoot_pt = ASHOOT_PT;
            aim_pt   = AAIMTO_PT;
        }

        Point2d park_pt = CTRPRKPT;
        if(autoStrategy == Field.AutoStrategy.SHOOT_PARKCRNR         ||
           autoStrategy == Field.AutoStrategy.ANGSHOOT_PARKCRNR      ||
           autoStrategy == Field.AutoStrategy.SHOOT_PUSH_PARKCRNR    ||
           autoStrategy == Field.AutoStrategy.ANGSHOOT_PUSH_PARKCRNR)
        {
            park_pt = CRNPRKPT;
        }

        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);

        //SHOOT PTS
        points.add(start_pt);
        segDirs.add(Segment.SegDir.FORWARD);
        segSpeeds.add(0.3);
        actions.add(Segment.Action.NOTHING);
        points.add(presh_pt);
        segDirs.add(Segment.SegDir.FORWARD);
        segSpeeds.add(0.3);
        actions.add(Segment.Action.SHOOT);
        points.add(shoot_pt);

        switch (autoStrategy) //setup PUSH points/actions
        {
            case ANGSHOOT_PARKCNTR:
            case ANGSHOOT_PARKCRNR:
            case SHOOT_PARKCNTR:
            case SHOOT_PARKCRNR:
                break;

            case ANGSHOOT_PUSH_PARKCNTR:
            case ANGSHOOT_PUSH_PARKCRNR:
            case SHOOT_PUSH_PARKCNTR:
            case SHOOT_PUSH_PARKCRNR:
            {
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Segment.Action.SCAN_IMAGE);
                points.add(SCAN1_PT); //2nd turn - scan images
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(0.5);
                actions.add(Segment.Action.FIND_BEACON);
                points.add(BECN1_PT); //2nd turn - find beacon
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(APP_SPEED);
                actions.add(Segment.Action.NOTHING);
                points.add(PRSS1_PT); //Push button
                segDirs.add(Segment.SegDir.REVERSE);
                segSpeeds.add(REV_SPEED);
                actions.add(Segment.Action.RST_PUSHER);
                points.add(SCAN1_PT); //Rev to clear pt for turn
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Segment.Action.SCAN_IMAGE);
                points.add(SCAN2_PT); //scan for images
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(0.5);
                actions.add(Segment.Action.FIND_BEACON);
                points.add(BECN2_PT); //find beacon
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(APP_SPEED);
                actions.add(Segment.Action.NOTHING);
                points.add(PRSS2_PT); //push button
                segDirs.add(Segment.SegDir.REVERSE);
                segSpeeds.add(REV_SPEED);
                actions.add(Segment.Action.RST_PUSHER);
                points.add(SCAN2_PT); //Rev to clear for turn
                break;
            }
        }

        //PARK PTS
        segDirs.add(Segment.SegDir.FORWARD);
        segSpeeds.add(DEF_SPEED);
        actions.add(Segment.Action.NOTHING);
        points.add(park_pt);

        return points;
    }

    Points(Field.AutoStrategy autoStrategy)
    {
        this.autoStrategy = autoStrategy;

        Vector<Point2d> pts = initPoints();
        Vector<Point2d> redPoints  = initRedPoints(pts);
        Vector<Point2d> bluePoints = initBluePoints(pts);

        redSegs  = initSegments(redPoints);
        blueSegs = initSegments(bluePoints);

        redTurns  = initTurns(redSegs);
        blueTurns = initTurns(blueSegs);
    }

    final Segment[] getSegments(Field.Alliance color)
    {
        segments = redSegs;
        if (color == Field.Alliance.BLUE)
        {
            segments = blueSegs;
        }

        return segments;
    }

    final double[] getTurns(Field.Alliance color)
    {
        turns = redTurns;
        if (color == Field.Alliance.BLUE)
        {
            turns = blueTurns;
        }
        return turns;
    }

    final Vector<Segment.Action> getActions()
    {
        return actions;
    }

    final Vector<Double> getSegSpeeds()
    {
        return segSpeeds;
    }

    final Vector<Segment.SegDir> getSegDirs()
    {
        return segDirs;
    }

    private Vector<Point2d> initRedPoints(Vector<Point2d> inpts)
    {
        Vector<Point2d> rpts = new Vector<>(inpts.size());

        for(Point2d rpt : inpts)
        {
            rpts.add(rpt);
        }
        return rpts;
    }

    private Vector<Point2d> initBluePoints(Vector<Point2d> inpts)
    {
        Vector<Point2d> bpts = new Vector<>(inpts.size());

        for(Point2d rpt : inpts)
        {
            bpts.add(convertRtoB(rpt));
        }
        return bpts;
    }

    private Segment[] initSegments(Vector<Point2d> pts)
    {
        int numSegs = pts.size() - 1;
        Segment[] pathSegs = new Segment[numSegs];
        Segment seg;
        for(int s = 0; s < numSegs; ++s)
        {
            seg = new Segment("DRV" + s, pts.get(s), pts.get(s+1));
            seg.setDir(segDirs.get(s));
            seg.setSpeed(segSpeeds.get(s));
            seg.setAction(actions.get(s));
            pathSegs[s] = seg;
        }

        return pathSegs;
    }

    @SuppressWarnings("unused")
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

        double inturns[] = new double[numTrns];

        for(int t = 0; t < numTrns; ++t)
        {
            Segment s1 = segs[t];
            Segment s2 = segs[t+1];
            double h1 = s1.getFieldHeading();
            if(s1.getDir()   == Segment.SegDir.REVERSE) h1+=180.0;
            double h2 = s2.getFieldHeading();
            if(s2.getDir() == Segment.SegDir.REVERSE) h2+=180.0;

            double ang = h2 - h1;

            while(ang >   180.0) ang -= 360.0;
            while(ang <= -180.0) ang += 360.0;
            inturns[t] = ang;

            System.out.println(
                    String.format(Locale.US,
                            "Seg %2d %s %s %s Fhdg: %6.2f Len: %6.2f %s",
                               t, s1.getName(), s1.getStrtPt(), s1.getTgtPt(),
                               h1, s1.getLength(), s1.getDir()));

            System.out.println(
                    String.format(Locale.US,
                            "Seg %2d %s %s %s Fhdg: %6.2f Len: %6.2f",
                            t+1, s1.getName(), s2.getStrtPt(), s2.getTgtPt(),
                            h2, s2.getLength()));
            System.out.println(String.format(Locale.US, "%6.2f", ang));
        }
        return inturns;
    }

    private Point2d convertRtoB(Point2d rpt)
    {
        double bx = -rpt.getY();
        double by = -rpt.getX();
        return new Point2d(bx,by);
    }
    
    public String toString()
    {
        StringBuilder sbldr = new StringBuilder();
        for (int i=0; i<segments.length; i++)
        {
            sbldr.append(segments[i].toString()).append("\n");
//            if(i<turns.length)
//            {
//                sbldr.append("  turn: ").append(turns[i]).append("\n");
//            }

            sbldr.append("  speed: ").append(segSpeeds.get(i)).append("\n");
            sbldr.append("  action: ").append(actions.get(i)).append("\n");
        }
        return sbldr.toString();
    }

    private final static double REAR_OFFSET = ShelbyBot.REAR_OFFSET;
    private final static double FRNT_OFFSET = ShelbyBot.FRNT_OFFSET;

    private static final double S_WALL = Field.S_WALL_Y;
    private static final double W_WALL = Field.W_WALL_X;

    private static final double STARTX  = -1*12;
    private static final double STARTY  =  S_WALL + REAR_OFFSET;
    private static final double PRSHTY  =  -64.0;
    private static final double SHOOTY  =  -60.5;
    private static final double AIMTOX  =  -1*12;
    private static final double AIMTOY  =  -4*12;

    private static final double ASTARTX =  1.0*12;
    private static final double ASHOOTX =  8.9;
    private static final double ASHOOTY =  -56.6;
    private static final double AAIMTOX =  9.0;
    private static final double AAIMTOY =  -48.0;

    private static final double TRGT1_Y = -12.0;
    private static final double TRGT2_Y =  36.0;
    private static final double CTRPRKX = -12.0;
    private static final double CTRPRKY = -12.0;
    private static final double CRNPRKX = -48.0;
    private static final double CRNPRKY = -48.0;

    private static final double SAFETY  = 1.0;
    private static final double SCAN_X  = -30.0;
    private static final double BECN_X  = -48.0;
    private static final double TOUCHX  = W_WALL + FRNT_OFFSET + SAFETY;

    private Point2d START_PT = new Point2d(STARTX, STARTY);
    private Point2d PRSHT_PT = new Point2d(STARTX, PRSHTY);
    private Point2d SHOOT_PT = new Point2d(STARTX, SHOOTY);
    private Point2d AIMTO_PT = new Point2d(AIMTOX, AIMTOY);

    private Point2d ASTART_PT = new Point2d(ASTARTX, STARTY);
    private Point2d APRSHT_PT = new Point2d(ASTARTX, PRSHTY);
    private Point2d ASHOOT_PT = new Point2d(ASHOOTX, ASHOOTY);
    private Point2d AAIMTO_PT = new Point2d(AAIMTOX, AAIMTOY);

    private Point2d SCAN1_PT = new Point2d(SCAN_X, TRGT1_Y);
    private Point2d BECN1_PT = new Point2d(BECN_X, TRGT1_Y);
    private Point2d PRSS1_PT = new Point2d(TOUCHX, TRGT1_Y);
    private Point2d SCAN2_PT = new Point2d(SCAN_X, TRGT2_Y);
    private Point2d BECN2_PT = new Point2d(BECN_X, TRGT2_Y);
    private Point2d PRSS2_PT = new Point2d(TOUCHX, TRGT2_Y);

    private Point2d CTRPRKPT = new Point2d(CTRPRKX, CTRPRKY);
    private Point2d CRNPRKPT = new Point2d(CRNPRKX, CRNPRKY);

    private final static int    MAX_SEGMENTS = 16;

    private Segment[] segments;
    private Segment[] redSegs;
    private Segment[] blueSegs;

    private double[] turns;
    private double[] redTurns;
    private double[] blueTurns;

    private Vector<Segment.Action> actions = new Vector<>(MAX_SEGMENTS);
    private Vector<Double> segSpeeds = new Vector<>(MAX_SEGMENTS);
    private Vector<Segment.SegDir> segDirs = new Vector<>(MAX_SEGMENTS);

    private Field.AutoStrategy autoStrategy =
            Field.AutoStrategy.SHOOT_PUSH_PARKCNTR;

    @SuppressWarnings("FieldCanBeLocal")
    private static double DEF_SPEED = 0.7;
    @SuppressWarnings("FieldCanBeLocal")
    private static double REV_SPEED = 0.4;
    @SuppressWarnings("FieldCanBeLocal")
    private static double APP_SPEED = 0.2;

    public static void main(String[] args)
    {
        Points ps = new Points(Field.AutoStrategy.SHOOT_PUSH_PARKCNTR);

        Vector<Segment.Action> acts = ps.getActions();
        Vector<Double> spds = ps.getSegSpeeds();
        double[] trns = ps.getTurns(Field.Alliance.RED);
        Segment[] segs = ps.getSegments(Field.Alliance.RED);

        System.out.print(ps.toString());
    }
}
