package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import ftclib.FtcOpMode;

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

    public void move(double pwr)
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
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void stopAndReset()
    {
        stopMotion();
        resetCounts();
        //time.reset();
    }

    public void driveDistance(double dst, double pwr, Direction dir)
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

    void driveDistanceLinear(double dst, double pwr, Direction dir)
    {
        driveDistance(dst, pwr, dir);

        noMoveTimer.reset();
        lposLast = left_drive.getCurrentPosition();
        rposLast = right_drive.getCurrentPosition();
        while(isBusy() &&
                      !areMotorsStuck() &&
                      op.opModeIsActive() &&
                      !op.isStopRequested())
        {
            op.idle();
        }

        left_drive.setPower(0.0);
        right_drive.setPower(0.0);
        DbgLog.msg("SJH: driveDistanceLinear ldc %6d rdc %6d",
                left_drive.getCurrentPosition(),
                right_drive.getCurrentPosition());

        stopAndReset();
    }

    void driveToPoint(Point2d tgtPt, double pwr, Direction dir)
    {
        if (tgtPt == null)  DbgLog.error("SJH tgtPt null in driveToPoint");
        if (currPt == null) DbgLog.error("SJH currPt null in driveToPoint");
        double dist = currPt.distance(tgtPt);
        driveDistance(dist, pwr, dir);
    }

    void driveToPointLinear(Point2d tgtPt, double pwr, Direction dir)
    {
        if (tgtPt == null)  DbgLog.error("SJH tgtPt null in driveToPoint");
        if (currPt == null) DbgLog.error("SJH currPt null in driveToPoint");
        driveToPoint(tgtPt, pwr, dir);

        int tHdg = getGryoFhdg();

        ptmr.reset();
        noMoveTimer.reset();
        lposLast = left_drive.getCurrentPosition();
        rposLast = right_drive.getCurrentPosition();
        DbgLog.msg("SJH: Starting drive corrections");
        while(isBusy() &&
              !areMotorsStuck() &&
              op.opModeIsActive() &&
              !op.isStopRequested())
        {
            if(gyroReady)
            {
                makeGyroCorrections(pwr, tHdg);
            }
            else
            {
                DbgLog.msg("SJH: GYRO NOT READY");
            }
            op.idle();

            if(ptmr.seconds() > 0.2) ptmr.reset();
        }

        left_drive.setPower(0.0);
        right_drive.setPower(0.0);
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

    boolean ctrTurnGyro(double hdg, double pwr)
    {
        double   error;
        double   steer;
        boolean  onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getGyroError((int)hdg);

        if (Math.abs(error) <= TURN_TOLERANCE)
        {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else
        {
            steer = getSteer(error, PADJ_TURN);
            rightSpeed  = pwr * steer;
            Range.clip(rightSpeed, -1, 1);
            if(Math.abs(rightSpeed) < minSpeed)
            {
                rightSpeed = Math.signum(rightSpeed) * minSpeed;
            }
            leftSpeed   = -rightSpeed;
        }

        left_drive.setPower(leftSpeed);
        right_drive.setPower(rightSpeed);

        if(ptmr.seconds() > 0.2)
        {
            ptmr.reset();
            DbgLog.msg("SJH: TGT %d INTZ %d ERR %d STR %4.2f L %4.2f R %4.2f",
                    (int) hdg, gyro.getIntegratedZValue(),
                    (int) error, steer, leftSpeed, rightSpeed);
        }
        return onTarget;
    }

    void ctrTurnLinear(double angle, double pwr)
    {
        ctrTurn(angle, pwr);
        Direction tdir = Direction.FORWARD;
        ptmr.reset();
        noMoveTimer.reset();
        lposLast = left_drive.getCurrentPosition();
        rposLast = right_drive.getCurrentPosition();
        DbgLog.msg("SJH: Starting turn corrections");
        while(isBusy() &&
              !areMotorsStuck() &&
              op.opModeIsActive() &&
              !op.isStopRequested())
        {
            makeCorrections(pwr, tdir);
            waitForTick(10);
            if(ptmr.seconds() > 0.2) ptmr.reset();
        }

        DbgLog.msg("SJH: ldc %6d rdc %6d",
                left_drive.getCurrentPosition(),
                right_drive.getCurrentPosition());

        stopAndReset();
    }

    private int getGryoFhdg()
    {
        int cHdg = gyro.getIntegratedZValue() +
                           (int)Math.round(initHdg);

        while (cHdg <= -180) cHdg += 360;
        while (cHdg >   180) cHdg -= 360;

        return cHdg;
    }

    void ctrTurnToHeading(double tgtHdg, double pwr)
    {
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tgtHdg = Math.round(tgtHdg);

        DbgLog.msg("SJH: GYRO TURN to HDG %d", (int)tgtHdg);

        ptmr.reset();
        noMoveTimer.reset();
        lposLast = left_drive.getCurrentPosition();
        rposLast = right_drive.getCurrentPosition();
        DbgLog.msg("SJH: Starting gyro turn corrections");
        while(!ctrTurnGyro(tgtHdg, pwr) &&
              !areMotorsStuck()         &&
              op.opModeIsActive()       &&
              !op.isStopRequested())
        {
            op.idle();
            frame++;
            if(ptmr.seconds() > 0.2) ptmr.reset();
        }
        stopAndReset();
    }

    void ctrTurnLinearGyro(double angle, double pwr)
    {
        angle = Math.round(angle);
        int tgtHdg = (int)angle + getGryoFhdg();
        while(tgtHdg > 180)   tgtHdg -= 360;
        while(tgtHdg <= -180) tgtHdg += 360;
        DbgLog.msg("SJH: GYRO TURN %d to HDG %d", (int)angle, tgtHdg);

        ctrTurnToHeading(tgtHdg, pwr);
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

    int distanceToCounts(double distance)
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

    void setInitHdg(double initHdg)
    {
        this.initHdg = initHdg;
    }

    void setOpMode(FtcOpMode op)
    {
        this.op = op;
    }

    void setGryoReady(boolean gryoReady)
    {
        this.gyroReady = gryoReady;
    }

    public void init(DcMotor lft_drv, DcMotor rgt_drv, ModernRoboticsI2cGyro gyro)
    {
        DbgLog.msg("SJH CPI: %5.2f", CPI);
        frame = 0;
        left_drive  = lft_drv;
        right_drive = rgt_drv;
        this.gyro = gyro;

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            op.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    void makeGyroCorrections(double pwr, int thdg)
    {
        double ldp; // = Math.abs(left_drive.getPower());
        double rdp; // = Math.abs(right_drive.getPower());

        double err = getGyroError(thdg);

        if (Math.abs(err) < TURN_TOLERANCE)
            return;

        double steer = getSteer(err, PADJ_TURN);
        //if (dir == Direction.REVERSE) steer *= -1;

        rdp = pwr - steer;
        ldp = pwr + steer;

        double max = Math.max(Math.abs(rdp), Math.abs(ldp));
        if (max > 1.0)
        {
            rdp = rdp / max;
            ldp = ldp / max;
        }

        if (ptmr.seconds() > 0.2)
        {
            DbgLog.msg("SJH %4d lpwr: %5.3f rpwr: %5.3f err: %5.3f str %5.3f rt %5.3f",
                    frame, ldp, rdp, err, steer, rt.seconds());
        }
    }

    void makeCorrections(double pwr, Direction dir)
    {
        double ldp; // = Math.abs(left_drive.getPower());
        double rdp; // = Math.abs(right_drive.getPower());

        double err = getEncoderError();

        if(Math.abs(err) > THRESH)
        {
            double steer = getSteer(err, PADJ);
            if(Math.abs(steer) > steer)  steer = Math.signum(steer) * pwr;
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
                left_drive.setPower(0.0);
                left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(Math.abs(diff) < 30)
                {
                    rdp = 0.0;
                    right_drive.setPower(0.0);
                    right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
            else if(Math.abs(rdc) >= Math.abs(tgtCnts))
            {
                rdp = 0.0;
                right_drive.setPower(0.0);
                right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(Math.abs(diff) < 30)
                {
                    ldp = 0.0;
                    left_drive.setPower(0.0);
                    left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
            else
            {
                move(ldp, rdp);
            }

            if (ptmr.seconds() > 0.2)
            {
                DbgLog.msg("SJH %4d ldc: %6d rdc: %6d diff: %2d " +
                                "lpwr: %5.3f rpwr: %5.3f err: %5.3f str %5.3f rt %5.3f",
                        frame, ldc, rdc, diff, ldp, rdp, err, steer, rt.seconds());
            }

            frame++;
        }
    }

    private double getEncoderError()
    {
        int ldc = Math.abs(left_drive.getCurrentPosition());
        int rdc = Math.abs(right_drive.getCurrentPosition());

        //convert LR count difference to angle
        return countsToAngle(rdc - ldc, VEH_WIDTH);
    }

    private int getGyroError(int tgtHdg)
    {
        int robotError;
        int gHdg = getGryoFhdg();
        robotError = tgtHdg - gHdg;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
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

    boolean areMotorsStuck()
    {
        if(usePosStop)
        {
            int lc = Math.abs(left_drive.getCurrentPosition());
            int rc = Math.abs(right_drive.getCurrentPosition());
            double lp = Math.abs(left_drive.getPower());
            double rp = Math.abs(right_drive.getPower());

            //If power is above threshold and encoders aren't changing,
            //stop after noMoveTimeout
            if(noMoveTimer.seconds() > noMoveTimeout)
            {
                if ((lp >= 0.0 && Math.abs(lposLast - lc) < noMoveThresh) ||
                    (rp >= 0.0 && Math.abs(rposLast - rc) < noMoveThresh))
                {
                    DbgLog.msg("SJH: MOTORS HAVE POWER BUT AREN'T MOVING - STOPPING %4.2f %4.2f",
                            lp, rp);
                    return true;
                }
                lposLast = lc;
                rposLast = rc;
                noMoveTimer.reset();
            }
        }
        return false;
    }

    boolean isBusy()
    {
        if(op != null && (!op.opModeIsActive() || op.isStopRequested()))
        {
            return false;
        }

        if(ptmr.seconds() > 0.2)
        {
            DbgLog.msg("SJH: ldc %6d rdc %6d  mptimer: %4.2f",
                    left_drive.getCurrentPosition(),
                    right_drive.getCurrentPosition(),
                    noMoveTimer.seconds());
        }

        return (left_drive.isBusy() && right_drive.isBusy());   //true if both are busy
        //return (left_drive.isBusy() || right_drive.isBusy()); //true if 1 is busy
    }

    public void setDrvTuner(double dtnr)
    {
        DRV_TUNER = dtnr;
        WHL_DIAMETER = 6.6 * DRV_TUNER;
        CIRCUMFERENCE = Math.PI * WHL_DIAMETER;
        CPI = ENCODER_CPR * GEAR_RATIO / CIRCUMFERENCE;
    }

    private static double DRV_TUNER = 1.00;
    private final static double TRN_TUNER = 1.0;
    private final static double TURN_TOLERANCE = 1.0;

    private final static double VEH_WIDTH   = ShelbyBot.BOT_WIDTH * TRN_TUNER;
    private static double WHL_DIAMETER = 6.35 * DRV_TUNER; //Diameter of the wheel (inches)
    private final static int    ENCODER_CPR = ShelbyBot.ENCODER_CPR;
    private final static double GEAR_RATIO  = 1;                   //Gear ratio

    private static double CIRCUMFERENCE = Math.PI * WHL_DIAMETER;
    private static double CPI = ENCODER_CPR * GEAR_RATIO / CIRCUMFERENCE;

    private static final double PADJ = 4.0;
    private static final double PADJ_TURN = 0.025;
    private static final double THRESH = Math.toRadians(0.004);

    public enum Direction {FORWARD, REVERSE}

    private DcMotor left_drive;
    private DcMotor right_drive;
    public ModernRoboticsI2cGyro gyro;

    private Point2d currPt = new Point2d(0.0, 0.0);
    private double initHdg = 0.0;

    private int frame   = 0;
    //private LinearOpMode lom;
    private ElapsedTime period  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime rt = new ElapsedTime();
    private ElapsedTime ptmr = new ElapsedTime();

    private double lposLast;
    private double rposLast;

    private double noMoveTimeout = 0.75;
    private int noMoveThresh = 10;
    private ElapsedTime noMoveTimer = new ElapsedTime();

    private double minSpeed = 0.04;

    private FtcOpMode op = null;

    private boolean usePosStop = true;

    private boolean gyroReady = false;

}
