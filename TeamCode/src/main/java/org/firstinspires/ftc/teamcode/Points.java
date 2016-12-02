package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;

import java.util.Vector;

class Points
{
    private Vector<Point2d> initPoints()
    {
        Point2d start_pt = START_PT;
        Point2d presh_pt = PRSHT_PT;
        Point2d shoot_pt = SHOOT_PT;

        if(startPos == Field.StartPos.START_B)
        {
            start_pt = ASTART_PT;
            presh_pt = APRSHT_PT;
            shoot_pt = ASHOOT_PT;
        }

        Point2d park_pt = CTRPRKPT;
        if(parkChoice == Field.ParkChoice.CORNER_PARK)
        {
            park_pt = CRNPRKPT;
        }

        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);

        //convenience declarations to make call params shorter
        Segment.SegDir fwd = Segment.SegDir.FORWARD;
        Segment.SegDir rev = Segment.SegDir.REVERSE;
        Segment.Action none   = Segment.Action.NOTHING;
        Segment.Action shoot  = Segment.Action.SHOOT;
        Segment.Action scan   = Segment.Action.SCAN_IMAGE;
        Segment.Action beacon = Segment.Action.FIND_BEACON;
        Segment.Action push   = Segment.Action.PUSH;
        Segment.Action reset  = Segment.Action.RST_PUSHER;

        //SHOOT PTS
        points.add(start_pt);
        if(startPos == Field.StartPos.START_B)
        {
            addPoint(points, fwd, 0.3, 1.00, Segment.TargetType.ENCODER, none, presh_pt);
        }
        addPoint(points, fwd, 0.6, 1.00, Segment.TargetType.ENCODER,  shoot, shoot_pt);

        if(pushChoice == Field.BeaconChoice.NEAR ||
           pushChoice == Field.BeaconChoice.BOTH)
        {
            if(useColor)
            {
                addPoint(points, fwd, 1.0,  1.00, Segment.TargetType.ENCODER, scan, PREP1_PT);
                addPoint(points, fwd, 0.15,  1.00, Segment.TargetType.COLOR, beacon, BECN1_PT);
            }
            else
            {
                addPoint(points, fwd, 0.8, 1.00, Segment.TargetType.ENCODER,   scan, SCAN1_PT);
                addPoint(points, fwd, 0.3, 1.00, Segment.TargetType.ENCODER, beacon, BECN1_PT);
            }

            addPoint(points, fwd, 0.3, 1.00, Segment.TargetType.ENCODER,   push, PRSS1_PT);
            addPoint(points, rev, 0.8, 1.00, Segment.TargetType.ENCODER,  reset, RVRS1_PT);
        }

        if(pushChoice == Field.BeaconChoice.FAR && startPos == Field.StartPos.START_B)
        {
            addPoint(points, fwd, 0.9, 1.00, Segment.TargetType.ENCODER, none, B_MID_PT);
        }

        if(pushChoice == Field.BeaconChoice.FAR ||
           pushChoice == Field.BeaconChoice.BOTH)
        {
            if(useColor)
            {
                addPoint(points, fwd, 0.9, 1.00, Segment.TargetType.ENCODER,     scan, PREP2_PT);
                addPoint(points, fwd, 0.15, 1.00, Segment.TargetType.COLOR, beacon, BECN2_PT);
            }
            else
            {
                addPoint(points, fwd, 0.8, 1.00, Segment.TargetType.ENCODER,   scan, SCAN2_PT);
                addPoint(points, fwd, 0.3, 1.00, Segment.TargetType.ENCODER, beacon, BECN2_PT);
            }

            addPoint(points, fwd, 0.3, 1.00, Segment.TargetType.ENCODER,   push, PRSS2_PT);
            addPoint(points, rev, 0.8, 1.00, Segment.TargetType.ENCODER,  reset, RVRS2_PT);
        }

        //PARK PTS
        addPoint(points, fwd, 1.0, 1.00, Segment.TargetType.ENCODER,   none, park_pt);

        return points;
    }

    Points(Field.StartPos startPos,
           Field.Alliance alliance,
           Field.BeaconChoice pushChoice,
           Field.ParkChoice parkChoice)
    {
        this.startPos     = startPos;
        this.alliance     = alliance;
        this.pushChoice   = pushChoice;
        this.parkChoice   = parkChoice;

        Vector<Point2d> pts = initPoints();
        Vector<Point2d> points;
        if(alliance == Field.Alliance.RED)
        {
            points = initRedPoints(pts);
        }
        else
        {
            points = initBluePoints(pts);
        }

        segments  = initSegments(points);
    }

    private void addPoint(Vector<Point2d> points,
                          Segment.SegDir dir,
                          double speed,
                          double tune,
                          Segment.TargetType targetType,
                          Segment.Action action,
                          Point2d pt)
    {
        segDirs.add(dir);
        segSpeeds.add(speed);
        ttypes.add(targetType);
        actions.add(action);
        points.add(pt);
    }

    final Segment[] getSegments()
    {
        return segments;
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
            String sname = pts.get(s+1).getName();

            seg = new Segment(sname, pts.get(s), pts.get(s+1));
            seg.setDir(segDirs.get(s));
            seg.setSpeed(segSpeeds.get(s));
            seg.setAction(actions.get(s));
            seg.setTgtType(ttypes.get(s));

            DbgLog.msg("SJH: setting up segment %s %s %s %4.1f tune: %4.2f",
                    seg.getName(), seg.getStrtPt(), seg.getTgtPt(),
                    seg.getFieldHeading(), seg.getDrvTuner());

            pathSegs[s] = seg;
        }

        for(int s = 0; s < pathSegs.length - 1 ; s++)
        {
            Segment curSeg = pathSegs[s];
            Segment nxtSeg = pathSegs[s+1];
            String sname = curSeg.getName();

            if(sname.equals("SCAN1") || sname.equals("SCAN2") ||
               sname.equals("BECN1") || sname.equals("BECN2"))
            {
                double nfhdg = nxtSeg.getFieldHeading();
                curSeg.setPostTurn(nfhdg);
                DbgLog.msg("SJH: Segment %s setting postTurn %4.2f", sname, nfhdg);
            }
            if(sname.equals("SHOOT") || sname.equals("ASHOOT"))
            {
                if (alliance == Field.Alliance.BLUE )
                {
                    BASKET_PT = convertRtoB(BASKET_PT);
                }
                Segment aim = new Segment("AIM", curSeg.getTgtPt(), BASKET_PT);
                double nfhdg = aim.getFieldHeading();
                curSeg.setPostTurn(nfhdg);
                DbgLog.msg("SJH: Segment %s setting postTurn %4.2f", sname, nfhdg);
            }
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

    private Point2d convertRtoB(Point2d rpt)
    {
        double bx = -rpt.getY();
        double by = -rpt.getX();
        rpt.setX(bx);
        rpt.setY(by);
        return rpt;
        //return new Point2d("B"+rpt.getName(), bx, by);
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

            sbldr.append(" speed: ").append(segSpeeds.get(i));
            sbldr.append(" action: ").append(actions.get(i)).append("\n");
        }
        return sbldr.toString();
    }

    private enum STATES
    {
        TURN_START,
        MOVE_PRSHT,
        TURN_PRSHT,
        MOVE_SHOOT,
        SCAN1,
        BECN1,
        PRSS1,
        REVS1
    }

    private final static double DEF_DRV_TUNER = 1.0;

    private final static double REAR_OFFSET = ShelbyBot.REAR_OFFSET;
    private final static double FRNT_OFFSET = 13.5;

    private static final double S_WALL = Field.S_WALL_Y;
    private static final double W_WALL = Field.W_WALL_X;

    private static final double STARTX  =  -12.0;
    private static final double STARTY  =  -66.5;
    private static final double AIMERY  =  -64.0;
    private static final double SHOOTY  =  -60.5;
    private static final double AIMTOX  =  -12.0;
    private static final double AIMTOY  =  -10.5;

    private static final double ASTARTX =  12.0;
    private static final double ASHOOTX =   8.9;
    private static final double ASHOOTY = -56.6;

    private static final double TRGT1_Y = -12.0;

    private static final double TRGT2_Y =  36.0;
    private static final double PREP2_Y  = TRGT2_Y - 4.0;
    private static final double CTRPRKX = -12.0;
    private static final double CTRPRKY = -12.0;
    private static final double CRNPRKX = -48.0;
    private static final double CRNPRKY = -40.0;

    private static final double SAFETY  =   0.0;
    private static final double SCAN_X  = -40.0;
    private static final double BECN_X  = -50.5;
    private static final double TOUCHX  = -56.0;
    private static final double TOUCH2  = -58.0;
    private static final double BMID_X  = -24.0;
    private static final double BMID_Y  = -24.0;

    private static final double PCT     = 0.92;
    private static final double PREP1_X = PCT*(BECN_X  - STARTX) + STARTX;
    private static final double PREP1_Y = PCT*(TRGT1_Y - SHOOTY) + SHOOTY;

//    private static final double PREP1_X  = BECN_X + 4.0;
//    private static final double PREP1_Y  = TRGT1_Y - 5.1;

    private Point2d START_PT = new Point2d("START", STARTX, STARTY);
    private Point2d PRSHT_PT = new Point2d("PRSHT", STARTX, AIMERY);
    private Point2d SHOOT_PT = new Point2d("SHOOT", STARTX, SHOOTY);

    private Point2d BASKET_PT = new Point2d("BASKET", AIMTOX, AIMTOY);

    private Point2d ASTART_PT = new Point2d("ASTART", ASTARTX, STARTY);
    private Point2d APRSHT_PT = new Point2d("APRSHT", ASTARTX, AIMERY);
    private Point2d ASHOOT_PT = new Point2d("ASHOOT", ASHOOTX, ASHOOTY);

    private Point2d SCAN1_PT = new Point2d("SCAN1", SCAN_X, TRGT1_Y);
    private Point2d PREP1_PT = new Point2d("PREP1", PREP1_X, PREP1_Y);
    private Point2d BECN1_PT = new Point2d("BECN1", BECN_X, TRGT1_Y);
    private Point2d PRSS1_PT = new Point2d("PRSS1", TOUCHX, TRGT1_Y);
    private Point2d RVRS1_PT = new Point2d("RVRS1", BECN_X, TRGT1_Y);
    private Point2d SCAN2_PT = new Point2d("SCAN2", SCAN_X, TRGT2_Y);
    private Point2d PREP2_PT = new Point2d("PREP2", BECN_X, PREP2_Y);
    private Point2d BECN2_PT = new Point2d("BECN2", BECN_X, TRGT2_Y);
    private Point2d PRSS2_PT = new Point2d("PRSS2", TOUCH2, TRGT2_Y);
    private Point2d RVRS2_PT = new Point2d("RVRS2", BECN_X, TRGT2_Y);
    private Point2d B_MID_PT = new Point2d("B_MID", BMID_X, BMID_Y);

    private Point2d CTRPRKPT = new Point2d("CTRPRK", CTRPRKX, CTRPRKY);
    private Point2d CRNPRKPT = new Point2d("CRNPRK", CRNPRKX, CRNPRKY);

    private final static int    MAX_SEGMENTS = 16;

    private Segment[] segments;

    private Vector<Segment.Action> actions = new Vector<>(MAX_SEGMENTS);
    private Vector<Double> segSpeeds = new Vector<>(MAX_SEGMENTS);
    private Vector<Segment.SegDir> segDirs = new Vector<>(MAX_SEGMENTS);
    private Vector<Double> tuners = new Vector<>(MAX_SEGMENTS);
    private Vector<Segment.TargetType> ttypes = new Vector<>(MAX_SEGMENTS);

    private Field.StartPos     startPos   = Field.StartPos.START_A;
    private Field.BeaconChoice pushChoice = Field.BeaconChoice.NEAR;
    private Field.ParkChoice   parkChoice = Field.ParkChoice.CENTER_PARK;
    private Field.Alliance     alliance   = Field.Alliance.RED;
    private boolean            useColor   = true;
}
