package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

class Drivetrain
{
    private static void move(double lPwr, double rPwr)
    {
        left_drive.setPower(lPwr);
        right_drive.setPower(rPwr);
    }

    private static void move(double pwr)
    {
        move(pwr, pwr);
    }

    private static void stopMotion()
    {
        move(0.0, 0.0);
    }

/*    private void turnLeft(double pwr)
    {
        move(0.0, pwr);
    }

    private void turnRight(double pwr)
    {
        move(pwr, 0.0);
    }*/

    private static void resetCounts()
    {
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    static void stopAndReset()
    {
        stopMotion();
        resetCounts();
        //time.reset();
    }

    private static void driveDistance(double dst, double pwr, Direction dir)
    {
        int counts = distanceToCounts(dst);
        DbgLog.msg("SJH driveDistance: %6.2f Counts %d", dst, counts);

        if(dir == Direction.REVERSE)
        {
            counts*=-1;
        }

        int lft_target = left_drive.getCurrentPosition() + counts;
        int rgt_target = right_drive.getCurrentPosition() + counts;
        left_drive.setTargetPosition(lft_target);
        right_drive.setTargetPosition(rgt_target);

        //resetCounts();
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        move(pwr);
    }

    static void driveToPoint(Point2d tgtPt, double pwr, Direction dir)
    {
        if (tgtPt == null)  DbgLog.error("SJH tgtPt null in driveToPoint");
        if (currPt == null) DbgLog.error("SJH currPt null in driveToPoint");
        double dist = currPt.distance(tgtPt);
        driveDistance(dist, pwr, dir);
    }

    static void driveToPointLinear(Point2d tgtPt, double pwr, Direction dir)
    {
        if (tgtPt == null)  DbgLog.error("SJH tgtPt null in driveToPoint");
        if (currPt == null) DbgLog.error("SJH currPt null in driveToPoint");
        driveToPoint(tgtPt, pwr, dir);

        while(isBusy())
        {
            makeCorrections(pwr, dir);
        }

        DbgLog.msg("SJH: ldc %6d rdc %6d",
                left_drive.getCurrentPosition(),
                right_drive.getCurrentPosition());

        stopAndReset();
        currPt = tgtPt;
    }

    static void ctrTurn(double angle, double pwr)
    {
        //perform a turn about rear axle center
        //left turns are positive angles

        int counts = angleToCounts(angle, VEH_WIDTH/2.0);
        DbgLog.msg("SJH Angle: %5.2f Counts: %4d", angle, counts);

        int lft_target = left_drive.getCurrentPosition()  - counts;
        int rgt_target = right_drive.getCurrentPosition() + counts;
        left_drive.setTargetPosition(lft_target);
        right_drive.setTargetPosition(rgt_target);

        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //pwr *= dir;
        move(pwr, pwr);
    }

    static void ctrTurnLinear(double angle, double pwr)
    {
        ctrTurn(angle, pwr);
        while(isBusy())
        {
            makeCorrections(pwr, Direction.FORWARD);
        }

        DbgLog.msg("SJH: ldc %6d rdc %6d",
                left_drive.getCurrentPosition(),
                right_drive.getCurrentPosition());

        stopAndReset();
    }

//    static void turn(double angle, double pwr, double radius)
//    {
//        int dir = 1;
//        if (angle < 0) dir = -1;
//        double rl = radius - dir*VEH_WIDTH/2.0;
//        double rr = radius + dir*VEH_WIDTH/2.0;
//        int lcnts = angleToCounts(angle, radius - dir * VEH_WIDTH/2.0);
//        int rcnts = angleToCounts(angle, radius + dir * VEH_WIDTH/2.0);
//
//        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        int lft_target = left_drive.getCurrentPosition() + lcnts;
//        int rgt_target = right_drive.getCurrentPosition() + rcnts;
//        left_drive.setTargetPosition(lft_target);
//        right_drive.setTargetPosition(rgt_target);
//
//        double arl = Math.abs(rl);
//        double arr = Math.abs(rr);
//        double rad_ratio = Math.min(arl, arr) / Math.max(arl, arr);
//
//        double ipwr = pwr * rad_ratio;
//
//        if (arl >= arr)
//        {
//            move(ipwr, pwr);
//        }
//        else
//        {
//            move(pwr, ipwr);
//        }
//    }

    private static int distanceToCounts(double distance)
    {
        return (int)(distance * CPI);
    }

    private static int angleToCounts(double angle, double radius)
    {
        return distanceToCounts(Math.toRadians(angle) * radius);
    }

    private static double countsToAngle(int counts, double radius)
    {
        return (double)counts/(CPI*radius);
    }

    static void setCurrPt(Point2d curPt)
    {
        currPt = curPt;
    }

    public static void init(DcMotor lft_drv, DcMotor rgt_drv)
    {
        DbgLog.msg("SJH CPI: %5.2f", CPI);
        left_drive = lft_drv;
        right_drive = rgt_drv;
    }


    static void makeCorrections(double pwr, Direction dir)
    {
//      int ldc = left_drive.getCurrentPosition();
//      int rdc = right_drive.getCurrentPosition();
        double ldp; // = Math.abs(left_drive.getPower());
        double rdp; // = Math.abs(right_drive.getPower());

        double err = getDriveError();

        if(Math.abs(err) > THRESH)
        {
            double steer = getSteer(err, PADJ);
            int d = 1;
            if (dir == Direction.REVERSE) d = -1;
            steer *= d;
//          int diff = Math.abs(ldc) - Math.abs(rdc);
//          DbgLog.msg("SJH ldc: %6d rdc: %6d diff: %d lpwr: %5.3f rpwr: %5.3f err: %5.3f str %5.3f",
//                  ldc, rdc, diff, ldp, rdp, err, steer);

            rdp = pwr - steer;
            ldp = pwr + steer;

            double max = Math.max(Math.abs(rdp), Math.abs(ldp));
            if(max > 1.0)
            {
                rdp = rdp / max;
                ldp = ldp / max;
            }

//          DbgLog.msg("SJH New power. lpwr: %5.3f rpwr: %5.3f",
//                  ldp, rdp);

            move(ldp, rdp);
        }
    }

    private static double getDriveError()
    {
        int ldc = Math.abs(left_drive.getCurrentPosition());
        int rdc = Math.abs(right_drive.getCurrentPosition());

        //convert LR count difference to angle
        return countsToAngle(rdc - ldc, VEH_WIDTH);
    }

    private static double getSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }

    static void logDriveState()
    {
        boolean ldb = left_drive.isBusy();
        boolean rdb = right_drive.isBusy();
        int ldc = left_drive.getCurrentPosition();
        int rdc = right_drive.getCurrentPosition();
        double ldp = left_drive.getPower();
        double rdp = right_drive.getPower();

        DbgLog.msg("SJH " +
                " ldb:" + ldb + " rdb:" + rdb +
                " ldc:" + ldc + " rdc:" + rdc +
                " ldp:" + ldp + " rdp:" + rdp);
    }

    static boolean isBusy()
    {
        return (left_drive.isBusy() && right_drive.isBusy());   //true if both are busy
        //return (left_drive.isBusy() || right_drive.isBusy()); //true if 1 is busy
    }

    private final static double DRV_TUNER = 1.0;
    private final static double TRN_TUNER = 1.0;

    private final static double VEH_WIDTH   = ShelbyBot.BOT_WIDTH * TRN_TUNER;
    private final static double WHL_DIAMETER = 6.0625 * DRV_TUNER; //Diameter of the wheel (inches)
    private final static int    ENCODER_CPR = ShelbyBot.ENCODER_CPR;
    private final static double GEAR_RATIO  = 1;                   //Gear ratio

    private final static double CIRCUMFERENCE = Math.PI * WHL_DIAMETER;
    private final static double CPI = ENCODER_CPR * GEAR_RATIO / CIRCUMFERENCE;

    private static final double PADJ = 0.5;
    private static final double THRESH = Math.toRadians(1.0);

    public enum Direction {FORWARD, REVERSE}

    private static DcMotor left_drive;
    private static DcMotor right_drive;

    private static Point2d currPt = new Point2d(0.0, 0.0);
}
