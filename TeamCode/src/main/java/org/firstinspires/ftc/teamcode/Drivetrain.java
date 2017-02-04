package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

class Drivetrain
{
    Drivetrain()
    {
        rt.reset();
    }

    public void move(double lPwr, double rPwr)
    {
        robot.rightMotor.setPower(rPwr);
        robot.leftMotor.setPower(lPwr);
    }

    public void move(double pwr)
    {
        move(pwr, pwr);
    }

    private void stopMotion()
    {
        move(0.0, 0.0);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetCounts()
    {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastLcnt = 0;
        lastRcnt = 0;
    }

    void stopAndReset()
    {
        move(0.0, 0.0);
        resetCounts();
    }

    public void resetLastPos()
    {
        noMoveTimer.reset();
        lposLast = robot.leftMotor.getCurrentPosition();
        rposLast = robot.rightMotor.getCurrentPosition();
    }

    public void driveDistance(double dst, double pwr, Direction dir)
    {
        int counts = distanceToCounts(dst);
        curCntTgt = counts;
        DbgLog.msg("SJH driveDistance: %6.2f Counts %d", dst, counts);

        if(dir == Direction.REVERSE)
        {
            counts*=-1;
        }

        if(doStopAndReset) stopAndReset();

        trgtLpos = initLpos + counts;
        trgtRpos = initRpos + counts;
        setStartValues();

        robot.leftMotor.setTargetPosition(trgtLpos);
        robot.rightMotor.setTargetPosition(trgtRpos);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        move(pwr);
    }

    int driveDistanceLinear(double dst, double pwr, Direction dir)
    {
        resetLastPos();

        driveDistance(dst, pwr, dir);

        while(op.opModeIsActive()    &&
              !op.isStopRequested()  &&
              isBusy()               &&
              !areDriveMotorsStuck())
        {
            logData();
            estimatePosition();
        }

        stopMotion();

        setEndValues();
        if(logOverrun) logOverrun(overtime);

        return((doneLpos - initLpos + doneRpos - initRpos)/2);
    }

    void driveToPoint(Point2d tgtPt, double pwr, Direction dir)
    {
        if (tgtPt == null)  DbgLog.error("SJH tgtPt null in driveToPoint");
        if (currPt == null) DbgLog.error("SJH currPt null in driveToPoint");
        double dist = currPt.distance(tgtPt);
        driveDistance(dist, pwr, dir);
    }

    int driveToPointLinear(Point2d tgtPt, double pwr, Direction dir, int targetHdg)
    {
        resetLastPos();

        initHdg = robot.getGyroFhdg();
        trgtHdg = targetHdg;

        DbgLog.msg("SJH: Starting drive");

        driveToPoint(tgtPt, pwr, dir);

        while(op.opModeIsActive() &&
              !op.isStopRequested() &&
              isBusy() &&
              !areDriveMotorsStuck())
        {
            logData();
            estimatePosition();

            makeGyroCorrections(pwr, trgtHdg);
        }

        stopMotion();
        setEndValues();
        if(logOverrun) logOverrun(overtime);

        setCurrPt(tgtPt);
        return ((doneLpos - initLpos + doneRpos - initRpos)/2);
    }

    int driveToPointLinear(Point2d tgtPt, double pwr, Direction dir)
    {
        return driveToPointLinear(tgtPt, pwr, dir, robot.getGyroFhdg());
    }

    void ctrTurnEncoder(double angle, double pwr)
    {
        //perform a turn about drive axle center
        //left turns are positive angles

        int counts = angleToCounts(angle, VEH_WIDTH/2.0);

        if(doStopAndReset) stopAndReset();

        trgtLpos = initLpos - counts;
        trgtRpos = initRpos + counts;
        trgtHdg = initHdg + (int) Math.round(angle);
        setStartValues();

        DbgLog.msg("SJH Angle: %5.2f Counts: %4d CHdg: %d", angle, counts, initHdg);

        robot.leftMotor.setTargetPosition(trgtLpos);
        robot.rightMotor.setTargetPosition(trgtRpos);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        move(pwr, pwr);
    }

    boolean ctrTurnGyro(double hdg, double pwr)
    {
        double   error;
        double   steer;
        boolean  onTarget = false;
        double leftSpeed;
        double rightSpeed;

        int ihdg = (int)(Math.round(hdg));

        // determine turn power based on +/- error
        error = getGyroError((int)hdg);

        if(error * lastGyroError < 0) //error sign changed - we passed target
        {
            DbgLog.msg("SJH: ctrTurnGryo overshot lastErr %5.2f error %5.2f hdg %d tgt %d",
                    lastGyroError, error, robot.getGyroFhdg(), ihdg);
        }

        if (Math.abs(error) <= TURN_TOLERANCE)
        {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else
        {
            double deltaErr = (error - lastGyroError)/gyroFrameTime.seconds();
            double d = Kd_GyroTurn * deltaErr;
            gyroFrameTime.reset();
            steer = getSteer(error, Kp_GyroTurn);
            rightSpeed  = pwr * steer;
            if(useDterm)
            {
                rightSpeed += d;
            }
            Range.clip(rightSpeed, -1, 1);
            if(Math.abs(rightSpeed) < minSpeed)
            {
                rightSpeed = Math.signum(rightSpeed) * minSpeed;
            }
            leftSpeed   = -rightSpeed;
        }

        move(leftSpeed, rightSpeed);
        lastGyroError = error;

        double ct = ptmr.seconds();
        if(ct > nextPrintTime)
        {
            nextPrintTime = ct + printTimeout;
            DbgLog.msg("SJH: TGT %d CHDG %d ERR %d STR %4.2f L %4.2f R %4.2f",
                    (int) hdg, robot.getGyroFhdg(),
                    (int) error, steer, leftSpeed, rightSpeed);
        }
        return onTarget;
    }

    void ctrTurnLinear(double angle, double pwr)
    {
        resetLastPos();

        DbgLog.msg("SJH: Starting turn of %f", angle);

        ctrTurnEncoder(angle, pwr);

        while(op.opModeIsActive() &&
              !op.isStopRequested() &&
              isBusy() &&
              !areMotorsStuck())
        {
            logData();
            estimatePosition();
            op.idle();
        }

        stopMotion();
        setEndValues();
        if(logOverrun) logOverrun(overtime);
    }

    void ctrTurnToHeading(double tgtHdg, double pwr)
    {
        tgtHdg = Math.round(tgtHdg);

        DbgLog.msg("SJH: GYRO TURN to HDG %d", (int)tgtHdg);

        if(doStopAndReset) stopAndReset();

        resetLastPos();

        trgtLpos = 0;
        trgtRpos = 0;
        trgtHdg = (int) tgtHdg;
        setStartValues();

        DbgLog.msg("SJH: Starting gyro turn");
        gyroFrameTime.reset();
        while(op.opModeIsActive()       &&
              !op.isStopRequested()     &&
              !ctrTurnGyro(tgtHdg, pwr) &&
              !areMotorsStuck())
        {
            logData();
            estimatePosition();
            op.idle();
            frame++;
        }

        stopMotion();
        setEndValues();
        if(logOverrun) logOverrun(overtime);
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
//        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        int lft_target = robot.leftMotor.getCurrentPosition() + lcnts;
//        int rgt_target = robot.rightMotor.getCurrentPosition() + rcnts;
//        robot.leftMotor.setTargetPosition(lft_target);
//        robot.rightMotor.setTargetPosition(rgt_target);
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

    double countsToDistance(double counts)
    {
        return (counts / CPI);
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
        DbgLog.msg("SJH: setCurrPt %s. estPos %s", curPt, estPos);
        currPt = curPt;
        if(numPts == 0)
        {
            xPos = currPt.getX();
            yPos = currPt.getY();
            estPos.setX(xPos);
            estPos.setY(yPos);
            if(robot.gyro != null) estHdg = robot.getGyroFhdg();
        }
        ++numPts;
    }

    public void estimatePosition()
    {
        int curLcnt =  robot.leftMotor.getCurrentPosition();
        int curRcnt = robot.rightMotor.getCurrentPosition();
        int dCntL = curLcnt - lastLcnt;
        int dCntR = curRcnt - lastRcnt;
        double dX = (dCntL)/CPI * Math.cos(Math.toRadians(robot.getGyroFhdg()));
        double dY = (dCntR)/CPI * Math.sin(Math.toRadians(robot.getGyroFhdg()));
        xPos += dX;
        yPos += dY;
        estPos.setX(xPos);
        estPos.setY(yPos);
        double dH = (dCntR-dCntL)/CPI/ShelbyBot.BOT_WIDTH;
        estHdg += dH;
        lastLcnt = curLcnt;
        lastRcnt = curRcnt;
    }

    void setStartHdg(double startHdg)
    {
        this.startHdg = startHdg;
    }

    void setOpMode(LinearOpMode op)
    {
        this.op = op;
    }

    void setDataLogger(DataLogger dl)
    {
        this.dl = dl;
    }

    public void init(ShelbyBot robot)
    {
        DbgLog.msg("SJH CPI: %5.2f", CPI);
        frame = 0;
        this.robot  = robot;
        this.gyro   = robot.gyro;

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        if(gyro == null || !robot.gyroReady) return;

        if (!op.opModeIsActive() || op.isStopRequested())
        {
            robot.rightMotor.setPower( 0.0 );
            robot.leftMotor.setPower( 0.0 );
            return;
        }

        double ldp; // = Math.abs(robot.leftMotor.getPower());
        double rdp; // = Math.abs(robot.rightMotor.getPower());

        double err = getGyroError(thdg);

        //if (Math.abs(err) < TURN_TOLERANCE)
        //   return;

        double steer = getSteer(err, Kp_GyrCorrection);
        //if (dir == Direction.REVERSE) steer *= -1;

        rdp = pwr + steer;
        ldp = pwr - steer;

        double max = Math.max(Math.abs(rdp), Math.abs(ldp));
        if (max > 1.0)
        {
            rdp = rdp / max;
            ldp = ldp / max;
        }

        robot.rightMotor.setPower( rdp );
        robot.leftMotor.setPower( ldp );

        double ct = ptmr.seconds();
        if (ct > nextPrintTime)
        {
            nextPrintTime = ct + printTimeout;
            DbgLog.msg("SJH %4d lpwr: %5.3f rpwr: %5.3f err: %5.3f str %5.3f rt %5.3f chdg %d thdg %d",
                    frame, ldp, rdp, err, steer, rt.seconds(), robot.getGyroFhdg(), thdg);
        }
    }

//    void makeCorrections(double pwr, Direction dir)
//    {
//        double ldp; // = Math.abs(robot.leftMotor.getPower());
//        double rdp; // = Math.abs(robot.rightMotor.getPower());
//
//        double err = getEncoderError();
//
//        if(Math.abs(err) > THRESH)
//        {
//            double steer = getSteer(err, Kp_EncCorrection);
//
//            if(Math.abs(steer) > pwr)  steer = Math.signum(steer) * pwr;
//            //if (dir == Direction.REVERSE) steer *= -1;
//
//            rdp = pwr ;//- steer;
//            ldp = pwr ;//+ steer;
//
//            double max = Math.max(Math.abs(rdp), Math.abs(ldp));
//            if(max > 1.0)
//            {
//                rdp = rdp / max;
//                ldp = ldp / max;
//            }
//
//            int ldc = robot.leftMotor.getCurrentPosition();
//            int rdc = robot.rightMotor.getCurrentPosition();
//            int diff = Math.abs(ldc) - Math.abs(rdc);
//            int tgtCnts = robot.leftMotor.getTargetPosition();
//
//            if(Math.abs(ldc) >= Math.abs(tgtCnts))
//            {
//                ldp = 0.0;
//                robot.leftMotor.setPower(0.0);
//                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                if(Math.abs(diff) < 30)
//                {
//                    rdp = 0.0;
//                    robot.rightMotor.setPower(0.0);
//                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                }
//            }
//            else if(Math.abs(rdc) >= Math.abs(tgtCnts))
//            {
//                rdp = 0.0;
//                robot.rightMotor.setPower(0.0);
//                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                if(Math.abs(diff) < 30)
//                {
//                    ldp = 0.0;
//                    robot.leftMotor.setPower(0.0);
//                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                }
//            }
//            else
//            {
//                move(ldp, rdp);
//            }
//
//            double ct = ptmr.seconds();
//            if (ct > nextPrintTime)
//            {
//                nextPrintTime = ct + printTimeout;
//                DbgLog.msg("SJH %4d ldc: %6d rdc: %6d diff: %2d " +
//                                "lpwr: %5.3f rpwr: %5.3f err: %5.3f str %5.3f rt %5.3f",
//                        frame, ldc, rdc, diff, ldp, rdp, err, steer, rt.seconds());
//            }
//
//            frame++;
//        }
//        else
//        {
//            ldp = pwr;
//            rdp = pwr;
//            move(ldp, rdp);
//        }
//    }

    private double getEncoderError()
    {
        int ldc = Math.abs(robot.leftMotor.getCurrentPosition());
        int rdc = Math.abs(robot.rightMotor.getCurrentPosition());

        //convert LR count difference to angle
        return countsToAngle(rdc - ldc, VEH_WIDTH);
    }

    private int getGyroError(int tgtHdg)
    {
        int robotError;
        int gHdg = robot.getGyroFhdg();
        robotError = tgtHdg - gHdg;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    private double getSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }

    private void setStartValues()
    {
        initLpos = robot.leftMotor.getCurrentPosition();
        initRpos = robot.rightMotor.getCurrentPosition();
        initHdg = robot.getGyroFhdg();
        if(logData)
        {
            dl.addField("BEGIN");
            dl.addField(initLpos);
            dl.addField(initRpos);
            dl.addField(initHdg);
            dl.addField(trgtLpos);
            dl.addField(trgtRpos);
            dl.addField(trgtHdg);
            dl.newLine();
        }
        DbgLog.msg("SJH: Begin ldc %6d rdc %6d Hdg %d",
                initLpos, initRpos, initHdg);
    }

    private void setEndValues()
    {
        doneLpos = robot.leftMotor.getCurrentPosition();
        doneRpos = robot.rightMotor.getCurrentPosition();
        doneHdg  = robot.getGyroFhdg();
        if(logData)
        {
            dl.addField("DONE");
            dl.addField(doneLpos);
            dl.addField(doneRpos);
            dl.addField(doneHdg);
            dl.newLine();
        }
        DbgLog.msg("SJH: End ldc %6d rdc %6d Hdg %d",
                doneLpos, doneRpos, doneHdg);
    }

    public void logOverrun(double t)
    {
        dl.addField("LOGGING OVERRUN");
        dl.newLine();
        ElapsedTime et = new ElapsedTime();
        while(et.seconds() < t)
        {
            logData();
            robot.waitForTick(10);
        }
    }

    boolean areMotorsStuck()
    {
        if(usePosStop)
        {
            int lc = Math.abs(robot.leftMotor.getCurrentPosition());
            int rc = Math.abs(robot.rightMotor.getCurrentPosition());
            double lp = Math.abs(robot.leftMotor.getPower());
            double rp = Math.abs(robot.rightMotor.getPower());

            //If power is above threshold and encoders aren't changing,
            //stop after noMoveTimeout
            if(noMoveTimer.seconds() > noMoveTimeout)
            {
                if ((lp >= 0.0 && Math.abs(lposLast - lc) < noMoveThreshLow) ||
                    (rp >= 0.0 && Math.abs(rposLast - rc) < noMoveThreshLow))
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

    boolean areDriveMotorsStuck()
    {
        if(usePosStop)
        {
            int lc = Math.abs(robot.leftMotor.getCurrentPosition());
            int rc = Math.abs(robot.rightMotor.getCurrentPosition());
            double lp = Math.abs(robot.leftMotor.getPower());
            double rp = Math.abs(robot.rightMotor.getPower());

            //If power is above threshold and encoders aren't changing,
            //stop after noMoveTimeout
            if(noMoveTimer.seconds() > noDriveMoveTimeout)
            {
                if ((lp >= 0.0 && Math.abs(lposLast - lc) < noMoveThreshLow) ||
                    (rp >= 0.0 && Math.abs(rposLast - rc) < noMoveThreshLow))
                {
                    DbgLog.msg("SJH: MOTORS HAVE POWER BUT AREN'T MOVING - STOPPING %4.2f %4.2f",
                            lp, rp);
                    return true;
                }
                if ((lp >= noMovePwrHi && Math.abs(lposLast - lc) < noMoveThreshHi) ||
                    (rp >= noMovePwrHi && Math.abs(rposLast - rc) < noMoveThreshHi))
                {
                    DbgLog.msg("SJH: MOTORS HAVE HI POWER BUT AREN'T MOVING - STOPPING %4.2f %4.2f",
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

        double ct = ptmr.seconds();
        if(ct > nextBusyPrintTime)
        {
            nextBusyPrintTime = ct + printTimeout;
            DbgLog.msg("SJH: ldc %6d rdc %6d  mptimer: %4.2f chdg %5d",
                    robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition(),
                    noMoveTimer.seconds(),
                    robot.getGyroFhdg());
        }


        boolean busy = false;
        if(busyAnd)
        {
            busy = robot.leftMotor.isBusy() && robot.rightMotor.isBusy(); //true if both are busy
        }
        else
        {
            busy = robot.leftMotor.isBusy() || robot.rightMotor.isBusy(); //true if 1 is busy
        }
        return busy;
    }

    public void setBusyAnd(boolean busyAnd)
    {
        this.busyAnd = busyAnd;
    }

    public void setDrvTuner(double dtnr)
    {
        CPI = ENCODER_CPR * GEAR_REDUC / (CIRCUMFERENCE * dtnr);
    }

    public void logData()
    {
//        double dlTimeout = 0.002;
//        if (datalogtimer.seconds() <dlTimeout) return;
//
//        datalogtimer.reset();

        double ldt = logDataTimer.seconds();
        if(logData && ldt > logTime)
        {
            logTime += logDataTimeout;
            if(robot.gyro != null) dl.addField(robot.getGyroFhdg());
            else                   dl.addField("");
            dl.addField(robot.leftMotor.getCurrentPosition());
            dl.addField(robot.rightMotor.getCurrentPosition());
            dl.addField("",robot.leftMotor.getPower());
            dl.addField("",robot.rightMotor.getPower());
            if(robot.colorSensor != null)
            {
                dl.addField(robot.colorSensor.red());
                dl.addField(robot.colorSensor.green());
                dl.addField(robot.colorSensor.blue());
            }
            dl.addField("", estPos.getX());
            dl.addField("", estPos.getY());
            dl.addField("", estHdg);
            dl.newLine();
        }
    }

    private static double DRV_TUNER = 1.00;
    private final static double TRN_TUNER = 1.0;
    private final static double TURN_TOLERANCE = 1.0;

    private final static double VEH_WIDTH   = ShelbyBot.BOT_WIDTH * TRN_TUNER;
    private static double WHL_DIAMETER = 4.25; //Diameter of the wheel (inches)
    private final static int    ENCODER_CPR = ShelbyBot.ENCODER_CPR;
    private final static double GEAR_REDUC = 0.5;                   //Gear ratio

    private static double CIRCUMFERENCE = Math.PI * WHL_DIAMETER;
    private static double CPI = ENCODER_CPR *GEAR_REDUC / (CIRCUMFERENCE * DRV_TUNER);

    private static final double Kp_GyrCorrection = 0.01;
    private static final double Kp_EncCorrection = 0.01;
    private static final double Kp_GyroTurn      = 0.01;
    private static final double Kd_GyroTurn      = 0.001;
    private static final double THRESH = Math.toRadians(0.004);

    public enum Direction {FORWARD, REVERSE}

    private ShelbyBot robot;
    public ModernRoboticsI2cGyro gyro;

    private Point2d currPt = new Point2d(0.0, 0.0);
    private double startHdg = 0.0;

    private int frame   = 0;
    //private LinearOpMode lom;
    private ElapsedTime period  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime rt = new ElapsedTime();
    private ElapsedTime ptmr = new ElapsedTime();

    private double lposLast;
    private double rposLast;

    private int initLpos;
    private int initRpos;
    private int trgtLpos;
    private int trgtRpos;
    private int doneLpos;
    private int doneRpos;
    private int overLpos;
    private int overRpos;

    private int initHdg;
    private int trgtHdg;
    private int doneHdg;
    private int overHdg;

    private double xPos = 0.0;
    private double yPos = 0.0;
    private Point2d estPos = new Point2d(xPos, yPos);
    private double  estHdg = 0.0;
    private int lastLcnt = 0;
    private int lastRcnt = 0;
    private int numPts = 0;

    private double noMoveTimeout = 0.5;
    private double noDriveMoveTimeout = 0.50;
    private int noMoveThreshLow = 10;
    private int noMoveThreshHi = 20;
    private double noMovePwrHi = 0.15;
    private ElapsedTime noMoveTimer = new ElapsedTime();

    private double printTimeout = 0.05;

    private double minSpeed = 0.1;

    private LinearOpMode op = null;

    private boolean usePosStop = true;
    private boolean doStopAndReset = false;

    private double lastGyroError = 0;
    private boolean useDterm = false;

    private ElapsedTime gyroFrameTime = new ElapsedTime();
    private ElapsedTime datalogtimer = new ElapsedTime();
    private DataLogger dl;
    private boolean logData = true;
    private double logDataTimeout = 0.005;
    private ElapsedTime logDataTimer = new ElapsedTime();
    private double logTime = logDataTimer.seconds();

    private int curCntTgt = 0;

    double nextPrintTime = ptmr.seconds();
    double nextBusyPrintTime = ptmr.seconds();

    private boolean logOverrun = true;
    private double overtime = 0.5;

    private boolean busyAnd = true;
}
