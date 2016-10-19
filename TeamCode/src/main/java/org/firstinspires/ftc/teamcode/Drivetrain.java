package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

class Drivetrain
{
    Drivetrain()
    {
        rt.reset();
    }

    private void move(double lPwr, double rPwr)
    {
        left_drive.setPower(lPwr);
        right_drive.setPower(rPwr);
    }

    private void move(double pwr)
    {
        move(pwr, pwr);
    }

    private void stopMotion()
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

    private void resetCounts()
    {
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void stopAndReset()
    {
        stopMotion();
        resetCounts();
        //time.reset();
    }

    private void driveDistance(double dst, double pwr, Direction dir)
    {
        int counts = distanceToCounts(dst);
        DbgLog.msg("SJH driveDistance: %6.2f Counts %d", dst, counts);

        if(dir == Direction.REVERSE)
        {
            counts*=-1;
        }

        int lft_target =  left_drive.getCurrentPosition() + counts;
        int rgt_target = right_drive.getCurrentPosition() + counts;
        left_drive.setTargetPosition(lft_target);
        right_drive.setTargetPosition(rgt_target);

        //resetCounts();
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        move(pwr);
    }

    void driveToPoint(Point2d tgtPt, double pwr, Direction dir)
    {
        if (tgtPt == null)  DbgLog.error("SJH tgtPt null in driveToPoint");
        if (currPt == null) DbgLog.error("SJH currPt null in driveToPoint");
        double dist = currPt.distance(tgtPt);
        if(dist < 6) pwr *= 0.2;
        driveDistance(dist, pwr, dir);
    }

    void driveToPointLinear(Point2d tgtPt, double pwr, Direction dir) throws InterruptedException
    {
        if (tgtPt == null)  DbgLog.error("SJH tgtPt null in driveToPoint");
        if (currPt == null) DbgLog.error("SJH currPt null in driveToPoint");
        driveToPoint(tgtPt, pwr, dir);

        while(isBusy())
        {
            makeCorrections(pwr, dir);
            waitForTick(20);
        }

        DbgLog.msg("SJH: ldc %6d rdc %6d",
                left_drive.getCurrentPosition(),
                right_drive.getCurrentPosition());

        stopAndReset();
        currPt = tgtPt;
    }

    void ctrTurn(double angle, double pwr)
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

    void ctrTurnLinear(double angle, double pwr) throws InterruptedException
    {
        ctrTurn(angle, pwr);
        Direction tdir = Direction.FORWARD;
        while(isBusy())
        {
            makeCorrections(pwr, tdir);
            waitForTick(20);
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

    private int distanceToCounts(double distance)
    {
        return (int)(distance * CPI);
    }

    private int angleToCounts(double angle, double radius)
    {
        return distanceToCounts(Math.toRadians(angle) * radius);
    }

    private double countsToAngle(int counts, double radius)
    {
        return (double)counts/(CPI*radius);
    }

    void setCurrPt(Point2d curPt)
    {
        currPt = curPt;
    }

    public void init(DcMotor lft_drv, DcMotor rgt_drv)
    {
        DbgLog.msg("SJH CPI: %5.2f", CPI);
        frame = 0;
        left_drive  = lft_drv;
        right_drive = rgt_drv;
        //lom = RedOpLinear.getInstance();
    }

    private void waitForTick(long periodMs) throws InterruptedException
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    void makeCorrections(double pwr, Direction dir)
    {
        double ldp; // = Math.abs(left_drive.getPower());
        double rdp; // = Math.abs(right_drive.getPower());

        double err = getDriveError();

        if(Math.abs(err) > THRESH)
        {
            double steer = getSteer(err, PADJ);
            //if (dir == Direction.REVERSE) steer *= -1;

            rdp = pwr - steer;
            ldp = pwr + steer;

            double max = Math.max(Math.abs(rdp), Math.abs(ldp));
            if(max > 1.0)
            {
                rdp = rdp / max;
                ldp = ldp / max;
            }

            int ldc = left_drive.getCurrentPosition();
            int rdc = right_drive.getCurrentPosition();
            int diff = Math.abs(ldc) - Math.abs(rdc);
            int tgtCnts = left_drive.getTargetPosition();

            if(Math.abs(ldc) >= Math.abs(tgtCnts))
            {
                ldp = 0.0;
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if(Math.abs(diff) < 30)
                {
                    rdp = 0.0;
                    right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }

            if(Math.abs(rdc) >= Math.abs(tgtCnts))
            {
                rdp = 0.0;
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if(Math.abs(diff) < 30)
                {
                    ldp = 0.0;
                    left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }

            move(ldp, rdp);

            if (frame%2 == 0)
            {
                DbgLog.msg("SJH %4d ldc: %6d rdc: %6d diff: %2d " +
                                "lpwr: %5.3f rpwr: %5.3f err: %5.3f str %5.3f rt %5.3f",
                        frame, ldc, rdc, diff, ldp, rdp, err, steer, rt.seconds());
            }

            frame++;
        }
    }

    private double getDriveError()
    {
        int ldc = Math.abs(left_drive.getCurrentPosition());
        int rdc = Math.abs(right_drive.getCurrentPosition());

        //convert LR count difference to angle
        return countsToAngle(rdc - ldc, VEH_WIDTH);
    }

    private double getSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }

    void logDriveState()
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

    boolean isBusy()
    {
        //return (left_drive.isBusy() && right_drive.isBusy());   //true if both are busy
        return (left_drive.isBusy() || right_drive.isBusy()); //true if 1 is busy
    }

    private final static double DRV_TUNER = 1.0;
    private final static double TRN_TUNER = 1.0;

    private final static double VEH_WIDTH   = ShelbyBot.BOT_WIDTH * TRN_TUNER;
    private final static double WHL_DIAMETER = 6.684 * DRV_TUNER; //Diameter of the wheel (inches)
    private final static int    ENCODER_CPR = ShelbyBot.ENCODER_CPR;
    private final static double GEAR_RATIO  = 1;                   //Gear ratio

    private final static double CIRCUMFERENCE = Math.PI * WHL_DIAMETER;
    private final static double CPI = ENCODER_CPR * GEAR_RATIO / CIRCUMFERENCE;

    private static final double PADJ = 4.0;
    private static final double THRESH = Math.toRadians(0.004);

    public enum Direction {FORWARD, REVERSE}

    private DcMotor left_drive;
    private DcMotor right_drive;

    private Point2d currPt = new Point2d(0.0, 0.0);

    private int frame   = 0;
    //private LinearOpMode lom;
    private ElapsedTime period  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime rt = new ElapsedTime();
}
