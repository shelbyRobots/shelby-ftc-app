package org.firstinspires.ftc.teamcode;

import java.util.Vector;

class BluePoints
{
    BluePoints()
    {
        points.add(new Point2d(STARTX,   STARTY));
        points.add(new Point2d(TGT_1_X,  STARTY));
        points.add(new Point2d(TGT_1_X,  TOUCH_Y));
        points.add(new Point2d(TGT_1_X,  CLEAR_Y));
        points.add(new Point2d(TGT_2_X,  CLEAR_Y));
        points.add(new Point2d(TGT_2_X,  TOUCH_Y));
        points.add(new Point2d(TGT_2_X,  CLEAR_Y));
        points.add(new Point2d(END_X,    END_Y));

        revSegs.add("DRV3");
        revSegs.add("DRV6");
    }

    private final static double REAR_OFFSET = ShelbyBot.REAR_OFFSET;
    private final static double FRNT_OFFSET = ShelbyBot.FRNT_OFFSET;

    private static final double N_WALL = Field.N_WALL_Y;
    private static final double E_WALL = Field.E_WALL_X;

    private static final double STARTX = E_WALL - REAR_OFFSET;
    private static final double STARTY = 2 * 12;
    private static final double TGT_1_X = Field.wheelsPos[0];
    private static final double TGT_2_X = Field.legosPos[0];
    private static final double END_X = 9.0;
    private static final double END_Y = 9.0;

    private static final double CLEAR  = 6;
    private static final double SAFETY = 3;
    private static final double CLEAR_Y = N_WALL - FRNT_OFFSET - CLEAR;
    private static final double TOUCH_Y = N_WALL - FRNT_OFFSET - SAFETY;

    private final static int    MAX_SEGMENTS = 16;
    static  final Vector<String>  revSegs = new Vector<>(MAX_SEGMENTS);
    static  final Vector<Point2d> points  = new Vector<>(MAX_SEGMENTS);
}