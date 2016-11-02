package org.firstinspires.ftc.teamcode;

import java.util.Locale;
import java.util.Vector;

class Points
{
    private Vector<Point2d> initPoints()
    {
        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);

        switch (autoStrategy)
        {
            case ANGSHOOT_PARKCNTR:
            {
                //TODO: CHANGE START and SHOOT PTS
                points.add(new Point2d(STARTX, STARTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SHOOT);
                points.add(new Point2d(STARTX,  SHOOTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(CTR_PRK_X, CTR_PRK_Y));
                break;
            }
            case ANGSHOOT_PARKCRNR:
            {
                //TODO: CHANGE START and SHOOT PTS
                points.add(new Point2d(STARTX, STARTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SHOOT);
                points.add(new Point2d(STARTX,  SHOOTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(CRN_PRK_X, CRN_PRK_Y));
                break;
            }
            case ANGSHOOT_PUSH_PARKCNTR:
            {
                //TODO: CHANGE START and SHOOT PTS
                points.add(new Point2d(STARTX, STARTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SHOOT);
                points.add(new Point2d(STARTX,  SHOOTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(STARTX,  TRN_1_Y)); //1st turn pt
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SCAN_IMAGE);
                points.add(new Point2d(SCAN_X,  TGT_1_Y)); //2nd turn - scan images
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.FIND_BEACON);
                points.add(new Point2d(BECN_X,  TGT_1_Y)); //2nd turn - find beacon
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(APP_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(TOUCH_X, TGT_1_Y)); //Push button
                segDirs.add(Segment.SegDir.REVERSE);
                segSpeeds.add(REV_SPEED);
                actions.add(Action.RST_PUSHER);
                points.add(new Point2d(SCAN_X,  TGT_1_Y)); //Rev to clear pt for turn
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SCAN_IMAGE);
                points.add(new Point2d(SCAN_X,  TGT_2_Y)); //scan for images
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.FIND_BEACON);
                points.add(new Point2d(BECN_X,  TGT_2_Y)); //find beacon
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(APP_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(TOUCH_X, TGT_2_Y)); //push button
                segDirs.add(Segment.SegDir.REVERSE);
                segSpeeds.add(REV_SPEED);
                actions.add(Action.RST_PUSHER);
                points.add(new Point2d(SCAN_X,  TGT_2_Y)); //Rev to clear for turn
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(CTR_PRK_X, CTR_PRK_Y));
                break;
            }
            case ANGSHOOT_PUSH_PARKCRNR:
            {
                //TODO: CHANGE START and SHOOT PTS
                points.add(new Point2d(STARTX, STARTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SHOOT);
                points.add(new Point2d(STARTX,  SHOOTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(STARTX,  TRN_1_Y)); //1st turn pt
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SCAN_IMAGE);
                points.add(new Point2d(SCAN_X,  TGT_1_Y)); //2nd turn - scan images
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.FIND_BEACON);
                points.add(new Point2d(BECN_X,  TGT_1_Y)); //2nd turn - find beacon
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(APP_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(TOUCH_X, TGT_1_Y)); //Push button
                segDirs.add(Segment.SegDir.REVERSE);
                segSpeeds.add(REV_SPEED);
                actions.add(Action.RST_PUSHER);
                points.add(new Point2d(SCAN_X,  TGT_1_Y)); //Rev to clear pt for turn
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SCAN_IMAGE);
                points.add(new Point2d(SCAN_X,  TGT_2_Y)); //scan for images
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.FIND_BEACON);
                points.add(new Point2d(BECN_X,  TGT_2_Y)); //find beacon
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(APP_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(TOUCH_X, TGT_2_Y)); //push button
                segDirs.add(Segment.SegDir.REVERSE);
                segSpeeds.add(REV_SPEED);
                actions.add(Action.RST_PUSHER);
                points.add(new Point2d(SCAN_X,  TGT_2_Y)); //Rev to clear for turn
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(CRN_PRK_X, CRN_PRK_Y));
                break;
            }
            case SHOOT_PARKCNTR:
            {
                points.add(new Point2d(STARTX, STARTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SHOOT);
                points.add(new Point2d(STARTX,  SHOOTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(CTR_PRK_X, CTR_PRK_Y));
                break;
            }
            case SHOOT_PARKCRNR:
            {
                points.add(new Point2d(STARTX, STARTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SHOOT);
                points.add(new Point2d(STARTX,  SHOOTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(CRN_PRK_X, CRN_PRK_Y));
                break;
            }
            case SHOOT_PUSH_PARKCNTR:
            {
                points.add(new Point2d(STARTX, STARTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SHOOT);
                points.add(new Point2d(STARTX,  SHOOTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(STARTX,  TRN_1_Y)); //1st turn pt
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SCAN_IMAGE);
                points.add(new Point2d(SCAN_X,  TGT_1_Y)); //2nd turn - scan images
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.FIND_BEACON);
                points.add(new Point2d(BECN_X,  TGT_1_Y)); //2nd turn - find beacon
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(APP_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(TOUCH_X, TGT_1_Y)); //Push button
                segDirs.add(Segment.SegDir.REVERSE);
                segSpeeds.add(REV_SPEED);
                actions.add(Action.RST_PUSHER);
                points.add(new Point2d(SCAN_X,  TGT_1_Y)); //Rev to clear pt for turn
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SCAN_IMAGE);
                points.add(new Point2d(SCAN_X,  TGT_2_Y)); //scan for images
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.FIND_BEACON);
                points.add(new Point2d(BECN_X,  TGT_2_Y)); //find beacon
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(APP_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(TOUCH_X, TGT_2_Y)); //push button
                segDirs.add(Segment.SegDir.REVERSE);
                segSpeeds.add(REV_SPEED);
                actions.add(Action.RST_PUSHER);
                points.add(new Point2d(SCAN_X,  TGT_2_Y)); //Rev to clear for turn
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(CTR_PRK_X, CTR_PRK_Y));
                break;
            }
            case SHOOT_PUSH_PARKCRNR:
            {
                points.add(new Point2d(STARTX, STARTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SHOOT);
                points.add(new Point2d(STARTX,  SHOOTY));
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(STARTX,  TRN_1_Y)); //1st turn pt
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SCAN_IMAGE);
                points.add(new Point2d(SCAN_X,  TGT_1_Y)); //2nd turn - scan images
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.FIND_BEACON);
                points.add(new Point2d(BECN_X,  TGT_1_Y)); //2nd turn - find beacon
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(APP_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(TOUCH_X, TGT_1_Y)); //Push button
                segDirs.add(Segment.SegDir.REVERSE);
                segSpeeds.add(REV_SPEED);
                actions.add(Action.RST_PUSHER);
                points.add(new Point2d(SCAN_X,  TGT_1_Y)); //Rev to clear pt for turn
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.SCAN_IMAGE);
                points.add(new Point2d(SCAN_X,  TGT_2_Y)); //scan for images
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.FIND_BEACON);
                points.add(new Point2d(BECN_X,  TGT_2_Y)); //find beacon
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(APP_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(TOUCH_X, TGT_2_Y)); //push button
                segDirs.add(Segment.SegDir.REVERSE);
                segSpeeds.add(REV_SPEED);
                actions.add(Action.RST_PUSHER);
                points.add(new Point2d(SCAN_X,  TGT_2_Y)); //Rev to clear for turn
                segDirs.add(Segment.SegDir.FORWARD);
                segSpeeds.add(DEF_SPEED);
                actions.add(Action.NOTHING);
                points.add(new Point2d(CRN_PRK_X, CRN_PRK_Y));

                break;
            }
        }

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

    final Vector<Action> getActions()
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
        for(int s = 0; s < numSegs; ++s)
        {
            pathSegs[s] = new Segment("DRV" + s, pts.get(s), pts.get(s+1));
        }

        for (int i =0 ; i < pathSegs.length; i++)
        {
            pathSegs[i].setDir(segDirs.get(i));
        }

        segments = pathSegs;
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

        turns = new double[numTrns];

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
            turns[t] = ang;

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
        return turns;
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
            if(i<turns.length)
            {
                sbldr.append("  turn: ").append(turns[i]).append("\n");
            }

            sbldr.append("  speed: ").append(segSpeeds.get(i)).append("\n");
            sbldr.append("  action: ").append(actions.get(i)).append("\n");
        }
        return sbldr.toString();
    }


    enum Action
    {
        NOTHING,
        SHOOT,
        SCAN_IMAGE,
        FIND_BEACON,
        RST_PUSHER
    }

    private final static double REAR_OFFSET = ShelbyBot.REAR_OFFSET;
    private final static double FRNT_OFFSET = ShelbyBot.FRNT_OFFSET;

    private static final double S_WALL = Field.S_WALL_Y;
    private static final double W_WALL = Field.W_WALL_X;

    private static final double STARTX  = -1*12;
    private static final double STARTY  =  S_WALL + REAR_OFFSET;
    private static final double SHOOTY  =  STARTY + 1.0;
    private static final double TRN_1_Y =  S_WALL + 2*12;
    private static final double TGT_1_Y = -1*12;
    private static final double TGT_2_Y =  3*12;
    private static final double CTR_PRK_X = -9.0;
    private static final double CTR_PRK_Y = -9.0;
    private static final double CRN_PRK_X = W_WALL + 1*12;
    private static final double CRN_PRK_Y = S_WALL + 1*12;

    private static final double SAFETY = 3;
    private static final double SCAN_X = W_WALL + 4*12;
    private static final double BECN_X = W_WALL + 2*12;
    private static final double TOUCH_X = W_WALL + FRNT_OFFSET + SAFETY;

    private final static int    MAX_SEGMENTS = 16;

    private Segment[] segments;
    private Segment[] redSegs;
    private Segment[] blueSegs;

    private double[] turns;
    private double[] redTurns;
    private double[] blueTurns;

    private Vector<Action> actions = new Vector<>(MAX_SEGMENTS);
    private Vector<Double> segSpeeds = new Vector<>(MAX_SEGMENTS);
    private Vector<Segment.SegDir> segDirs = new Vector<>(MAX_SEGMENTS);

    private Field.AutoStrategy autoStrategy =
            Field.AutoStrategy.SHOOT_PUSH_PARKCNTR;

    @SuppressWarnings("FieldCanBeLocal")
    private static double DEF_SPEED = 0.5;
    @SuppressWarnings("FieldCanBeLocal")
    private static double REV_SPEED = 0.4;
    @SuppressWarnings("FieldCanBeLocal")
    private static double APP_SPEED = 0.2;

    public static void main(String[] args)
    {
        Points ps = new Points(Field.AutoStrategy.SHOOT_PUSH_PARKCNTR);

        Vector<Points.Action> acts = ps.getActions();
        Vector<Double> spds = ps.getSegSpeeds();
        double[] trns = ps.getTurns(Field.Alliance.RED);
        Segment[] segs = ps.getSegments(Field.Alliance.RED);

        System.out.print(ps.toString());
    }
}
