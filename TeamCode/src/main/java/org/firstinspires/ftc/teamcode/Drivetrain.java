package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

class Drivetrain
{
    Drivetrain()
    {
        rt.reset();
    }

    public void moveInit(double lPwr, double rPwr)
    {
        resetLastPos();
        move(lPwr, rPwr);
        logData(true, "INIT PWR SET");
    }

    public void move(double lPwr, double rPwr)
    {
//        curLpower = Math.round(100*lPwr)/100;
//        curRpower = Math.round(100*rPwr)/100;

        curLpower = lPwr;
        curRpower = rPwr;

        if(!gangMotors)
        {
            if(useSpeedThreads)
            {
                lSpdTask.setMotor(robot.leftMotor);
                rSpdTask.setMotor(robot.rightMotor);
                lSpdTask.setSpeed(curLpower);
                rSpdTask.setSpeed(curRpower);
            }
            else
            {
                if(lFirst)
                {
                    robot.leftMotor.setPower(curLpower);
                    robot.rightMotor.setPower(curRpower);
                }
                else
                {
                    robot.rightMotor.setPower(curRpower);
                    robot.leftMotor.setPower(curLpower);
                }
            }
        }
    }

    public void move(double pwr)
    {
        move(pwr, pwr);
    }

    public void stopMotion()
    {
        move(0.0, 0.0);
        logData(true, "MOTORS STOPPED");
    }

    private void resetCounts()
    {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        op.idle();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        curLpos = 0;
        curRpos = 0;
        lastLcnt = 0;
        lastRcnt = 0;
    }

    void stopAndReset()
    {
        move(0.0, 0.0);
        logData(true, "MOTORS STOPPED - RESETTING");
    }

    public void resetLastPos()
    {
        accelTimer.reset();
        noMoveTimer.reset();
        lposLast = robot.leftMotor.getCurrentPosition();
        rposLast = robot.rightMotor.getCurrentPosition();
    }

    public void driveToTarget(double pwr, int thresh)
    {
        setBusyAnd(false);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotor.setTargetPosition(trgtLpos);
        robot.rightMotor.setTargetPosition(trgtRpos);
        setInitValues();
        logStartValues("DRIVE_TRGT " + curLpos + " " + curRpos + " - " + trgtLpos + " " + trgtRpos);
        moveInit(pwr, pwr);
        while(isBusy(thresh)         &&
              !op.isStopRequested()  &&
              !areDriveMotorsStuck())
        {
            setCurValues();
            logData();

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
        }

        stopMotion();
        setEndValues("DRIVE_TRGT");
    }

    public void driveDistance(double dst, double pwr, Direction dir)
    {
        int counts = distanceToCounts(dst);

        RobotLog.ii("SJH", "driveDistance: %6.2f Counts %d", dst, counts);

        if(dir == Direction.REVERSE)
        {
            counts*=-1;
        }

        setInitValues();
        trgtLpos = initLpos + counts;
        trgtRpos = initRpos + counts;
        String dDistStr = String.format(Locale.US, "DRIVE_DIST %4.2f", dst);
        logStartValues(dDistStr);

        robot.leftMotor.setTargetPosition(trgtLpos);
        robot.rightMotor.setTargetPosition(trgtRpos);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        moveInit(pwr, pwr);
    }

    int driveDistanceLinear(double dst, double pwr, Direction dir, int targetHdg, boolean useCol)
    {
        trgtHdg = targetHdg;

        RobotLog.ii("SJH", "Starting drive");
        if(doStopAndReset) stopAndReset();
        logData(true, "LINDST");

        double startPwr = 0.1;
        boolean foundLine = false;

        initLpower = startPwr;
        initRpower = startPwr;
        int pwrSteps = 5;
        double pwrIncr = (pwr - startPwr)/pwrSteps;

        //extend distance if doing a color find to make sure we reach line
        int colOverCnt = 0;
        if(useCol)
        {
            double colOverDist = 6.0;
            colOverCnt = distanceToCounts(colOverDist);
            dst += colOverDist;
        }

        driveDistance(dst, startPwr, dir);
        int linLpos = trgtLpos;
        int linRpos = trgtRpos;

        while(op.opModeIsActive()    &&
              !op.isStopRequested()  &&
              isBusy()               &&
              !areDriveMotorsStuck())
        {
            setCurValues();
            logData();

            double ppwr = pwr;
            double tmpPwr = (curLpower + curRpower)/2;

            if(rampUp)
            {
                if(tmpPwr < pwr) tmpPwr = Math.min(pwr, tmpPwr + pwrIncr);
                ppwr = tmpPwr;
            }

            if(rampDown)
            {
                int lcnt = curLpos;
                int rcnt = curRpos;
                int hiSlow  = 960;
                int midSlow = 480 + colOverCnt/4;
                int lowSlow = 240 + colOverCnt;
                int remaining = Math.abs(((trgtLpos - lcnt) + (trgtRpos - rcnt)) / 2);
                if (Math.abs(remaining) < hiSlow)  ppwr = Math.min(ppwr, 0.5);
                if (Math.abs(remaining) < midSlow) ppwr = Math.min(ppwr, 0.25);
                if (Math.abs(remaining) < lowSlow) ppwr = Math.min(ppwr, 0.09);
            }

            RobotLog.ii("SJH", "ppwr " + ppwr + " curLpower " + curLpower +
                               " curRpower " + curRpower + " pwrIncr " +  pwrIncr);

            int COLOR_THRESH = 30;
            double lRem = countsToDistance(Math.abs(trgtLpos - curLpos));
            double rRem = countsToDistance(Math.abs(trgtRpos - curRpos));
            double colOnDist = 24.0;
            if(useCol && !robot.colorEnabled && (lRem < colOnDist || rRem < colOnDist))
            {
                robot.turnColorOn();
            }

            if(useCol && (curRed + curGrn + curBlu) > COLOR_THRESH)
            {
                linLpos = curLpos;
                linRpos = curRpos;
                linLpos -= colGyroOffset;
                linRpos -= colGyroOffset;
                stopMotion();
                setEndValues("COLOR_FIND " + linLpos + " " + linRpos);
                RobotLog.ii("SJH", "FOUND LINE");
                foundLine = true;
                trgtLpos = linLpos;
                trgtRpos = linRpos;
                break;
            }
            else
            {
                makeGyroCorrections(ppwr, trgtHdg, dir);
            }

            if(!isBusy()) break;

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
        }

        int kludge = 160;
        if(useCol && !foundLine)
        {
            trgtLpos -= (colOverCnt + kludge);
            trgtRpos -= (colOverCnt + kludge);
        }

        if(useCol) robot.turnColorOff();

        setEndValues("LINDST");
        stopMotion();
        if(logOverrun) logOverrun(overtime);

        return((doneLpos - initLpos + doneRpos - initRpos)/2);
    }

    int driveDistanceLinear(double dst, double pwr, Direction dir, int targetHdg)
    {
        return driveDistanceLinear(dst, pwr, dir, targetHdg, false);
    }

    int driveDistanceLinear(double dst, double pwr, Direction dir)
    {
        return driveDistanceLinear(dst, pwr, dir, robot.getGyroFhdg());
    }

    int driveToPoint(Point2d tgtPt, double pwr, Direction dir, int targetHdg)
    {
        if (tgtPt == null)  RobotLog.ee("SJH", "tgtPt null in driveToPoint");
        if (currPt == null) RobotLog.ee("SJH", "currPt null in driveToPoint");
        double dist = currPt.distance(tgtPt);
        return driveDistanceLinear(dist, pwr, dir, targetHdg);
    }

    int driveToPointLinear(Point2d tgtPt, double pwr, Direction dir, int targetHdg)
    {
        int dist = driveToPoint(tgtPt, pwr, dir, targetHdg);

        setCurrPt(tgtPt);
        return dist;
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

        setInitValues();
        trgtLpos = initLpos - counts;
        trgtRpos = initRpos + counts;
        trgtHdg  = initHdg  + (int) Math.round(angle);
        while(trgtHdg >   180) trgtHdg -= 360;
        while(trgtHdg <= -180) trgtHdg += 360;
        String angStr = String.format(Locale.US, "ENC_TURN %4.2f", angle);
        logStartValues(angStr);

        RobotLog.ii("SJH", "Angle: %5.2f Counts: %4d CHdg: %d", angle, counts, initHdg);

        robot.leftMotor.setTargetPosition(trgtLpos);
        robot.rightMotor.setTargetPosition(trgtRpos);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        moveInit(pwr, pwr);
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
            RobotLog.ii("SJH", "ctrTurnGryo overshot lastErr %5.2f error %5.2f hdg %d tgt %d",
                    lastGyroError, error, curHdg, ihdg);
        }

        if (Math.abs(error) <= TURN_TOLERANCE)
        {
            if(!gyroFirstGood)
            {
                gyroFirstGood = true;
                gyroGoodCount = 0;
                gyroGoodTimer.reset();
            }
            gyroGoodCount++;
            logData(true, "GYRO GOOD " + gyroGoodCount + " TIME: " + gyroFrameTime.milliseconds());
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = gyroGoodTimer.milliseconds() > gyroTimeout;
        }
        else
        {
            gyroGoodCount = 0;
            gyroFirstGood = false;
            gyroGoodTimer.reset();
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
            if(Math.abs(rightSpeed) < minGyroTurnSpeed)
            {
                rightSpeed = Math.signum(rightSpeed) * minGyroTurnSpeed;
            }
            leftSpeed   = -rightSpeed;
        }

        move(leftSpeed, rightSpeed);
        lastGyroError = error;

        double ct = ptmr.seconds();
        if(ct > nextPrintTime)
        {
            nextPrintTime = ct + printTimeout;
            RobotLog.ii("SJH", "TGT %d CHDG %d ERR %d STR %4.2f L %4.2f R %4.2f",
                    (int) hdg, curHdg,
                    (int) error, steer, leftSpeed, rightSpeed);
        }
        return onTarget;
    }

    void ctrTurnLinear(double angle, double pwr, int thresh)
    {
        RobotLog.ii("SJH", "Starting turn of %f", angle);

        if(doStopAndReset) stopAndReset();
        logData(true, "ENC TURN");

        ElapsedTime tTimer = new ElapsedTime();

        double startPwr = 0.1;
        initLpower = startPwr;
        initRpower = startPwr;

        int pwrSteps = 5;
        double pwrLIncr = (pwr - initLpower)/pwrSteps;
        double pwrRIncr = (pwr - initRpower)/pwrSteps;

        ctrTurnEncoder(angle, startPwr);

        while(op.opModeIsActive() &&
              !op.isStopRequested() &&
              isBusy(thresh) &&
              !areMotorsStuck() &&
              tTimer.seconds() < turnTimeLimit)
        {
            setCurValues();
            logData();

            double ppwr = pwr;

            double tmpPwr = (curLpower + curRpower)/2;

            if(rampUp)
            {
                if(tmpPwr < pwr) tmpPwr = Math.min(pwr, tmpPwr + pwrLIncr);
                ppwr = tmpPwr;
            }

            if(rampDown)
            {
                int lcnt = curLpos;
                int rcnt = curRpos;
                int remaining = (Math.abs(trgtLpos - lcnt) + Math.abs(trgtRpos - rcnt)) / 2;

                if (Math.abs(remaining) < 960) ppwr = Math.min(ppwr, 0.5);
                if (Math.abs(remaining) < 480) ppwr = Math.min(ppwr, 0.25);
                if (Math.abs(remaining) < 240) ppwr = Math.min(ppwr, 0.10);
            }
            double lpwr = ppwr;
            double rpwr = ppwr;

            if(stopIndividualMotorWhenNotBusy)
            {
                if(!isMotorBusy(MotorSide.LEFT))  lpwr = 0.0;
                if(!isMotorBusy(MotorSide.RIGHT)) rpwr = 0.0;
            }
            else if(!isBusy(TURN_BUSYTHRESH))
            {
                lpwr = 0.0;
                rpwr = 0.0;
            }

            move(lpwr, rpwr);

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
        }
        setEndValues("ENC_TURN");
        stopMotion();
        if(logOverrun) logOverrun(overtime);
    }

    void ctrTurnLinear(double angle, double pwr)
    {
        ctrTurnLinear(angle, pwr, TURN_BUSYTHRESH);
    }

    void ctrTurnToHeading(double tgtHdg, double pwr)
    {
        tgtHdg = Math.round(tgtHdg);

        RobotLog.ii("SJH", "GYRO TURN to HDG %d", (int)tgtHdg);

        gyroFirstGood = false;

        if(doStopAndReset) stopAndReset();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moveInit(0.0, 0.0);

        setInitValues();
        trgtLpos = 0;
        trgtRpos = 0;
        trgtHdg = (int) tgtHdg;
        String gyroStr = String.format(Locale.US, "GYRO_TURN %4.1f", tgtHdg);
        logStartValues(gyroStr);

        ElapsedTime tTimer = new ElapsedTime();

        RobotLog.ii("SJH", "Starting gyro turn");
        gyroFrameTime.reset();
        while(op.opModeIsActive()       &&
              !op.isStopRequested()     &&
              !ctrTurnGyro(tgtHdg, pwr) &&
              !areMotorsStuck()         &&
              tTimer.seconds() < turnTimeLimit)
        {
            setCurValues();
            logData();

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
        }
        setEndValues("GYRO_TURN");
        stopMotion();
    }

    void turn(double angle, double pwr, double radius)
    {
        //radius is distance from ctr of bot to ctr of curve
        //it is positive toward the left side
        //A radius of 0 is a ctr turn
        //A radius of +w/2 pivots on the left wheel
        //A radius of -w/2 pivots on the right wheel

        double rl = radius - VEH_WIDTH/2.0;
        double rr = radius + VEH_WIDTH/2.0;
        int lcnts = angleToCounts(angle, rl);
        int rcnts = angleToCounts(angle, rr);

        setBusyAnd(false);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int lft_target = initLpos + lcnts;
        int rgt_target = initRpos + rcnts;

        setInitValues();
        trgtLpos = lft_target;
        trgtRpos = rgt_target;
        trgtHdg  = initHdg  + (int) Math.round(angle);
        while(trgtHdg >   180) trgtHdg -= 360;
        while(trgtHdg <= -180) trgtHdg += 360;
        logStartValues("ENC_CURVE");

        robot.leftMotor.setTargetPosition(lft_target);
        robot.rightMotor.setTargetPosition(rgt_target);

        double arl = Math.abs(rl);
        double arr = Math.abs(rr);
        double rad_ratio = Math.min(arl, arr) / Math.max(arl, arr);

        double opwr = pwr;
        double ipwr = opwr * rad_ratio;

        double lp = ipwr;
        double rp = opwr;

        if (arl > arr)
        {
            lp = opwr;
            rp = ipwr;
        }

        moveInit(lp, rp);

        while(op.opModeIsActive() &&
              isBusy(TURN_BUSYTHRESH) &&
              !areMotorsStuck())
        {
            setCurValues();
            logData();

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
        }
        move(0.0, 0.0);
    }

    int distanceToCounts(double distance)
    {
        return (int)(distance * CPI);
    }

    double countsToDistance(double counts)
    {
        return (counts / CPI);
    }

    public int angleToCounts(double angle, double radius)
    {
        return distanceToCounts(Math.toRadians(angle) * radius);
    }

    private double countsToAngle(int counts, double radius)
    {
        return (double)counts/(CPI*radius);
    }

    void setCurrPt(Point2d curPt, boolean resetEstPt)
    {
        RobotLog.ii("SJH", "setCurrPt %s. estPos %s", curPt, estPos);
        currPt = curPt;
        if(numPts == 0 || resetEstPt)
        {
            xPos = currPt.getX();
            yPos = currPt.getY();
            estPos.setX(xPos);
            estPos.setY(yPos);
            estHdg = initHdg;
        }
        ++numPts;
    }

    void setCurrPt(Point2d curPt)
    {
        setCurrPt(curPt, false);
    }

    public void estimatePosition()
    {
        int curLcnt = curLpos;
        int curRcnt = curRpos;
        double radHdg = Math.toRadians(curHdg);

        if(curDriveDir != robot.getDriveDir())
        {
            curDriveDir = robot.getDriveDir();
            lastLcnt = curLcnt;
            lastRcnt = curRcnt;
            logData(true, "DDIR=" + curDriveDir.toString());
        } 

        int dCntL = curLcnt - lastLcnt;
        int dCntR = curRcnt - lastRcnt;
        double dX = 0.5*(dCntL+dCntR)/DEF_CPI * Math.cos(radHdg);
        double dY = 0.5*(dCntL+dCntR)/DEF_CPI * Math.sin(radHdg);
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
        RobotLog.ii("SJH", "CPI: %5.2f", CPI);
        frame = 0;
        this.robot  = robot;

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        curDriveDir = robot.getDriveDir();
        lastDriveDir = curDriveDir;
    }

    public void start()
    {
        if(useSpeedThreads)
        {
            lftSpdThread = new Thread(lSpdTask);
            rgtSpdThread = new Thread(rSpdTask);
            lftSpdThread.setName("LeftSpeedThread");
            rgtSpdThread.setName("RightSpeedThread");
            lftSpdThread.start();
            rgtSpdThread.start();
        }
    }

    public void cleanup()
    {
        stopMotion();
        RobotLog.ii("SJH", "CLOSING Drivetrain");
        if(lftSpdThread != null) lftSpdThread.interrupt();
        if(rgtSpdThread != null) rgtSpdThread.interrupt();
    }

    public void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            op.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    void makeGyroCorrections(double pwr, int thdg, Direction dir)
    {
        if(robot.gyro == null || !robot.gyroReady) return;

        double ldp;
        double rdp;

        double err = getGyroError(thdg);

        double steer = getSteer(err, Kp_GyrCorrection);
        if (dir == Direction.REVERSE) steer *= -1;

        rdp = pwr + steer;
        ldp = pwr - steer;

        double max = Math.max(Math.abs(rdp), Math.abs(ldp));
        if (max > 1.0)
        {
            rdp = rdp / max;
            ldp = ldp / max;
        }

        if(Math.abs(ldp) < minSpeed)
        {
            ldp = Math.signum(ldp) * minSpeed;
        }

        if(Math.abs(rdp) < minSpeed)
        {
            rdp = Math.signum(rdp) * minSpeed;
        }

        if(stopIndividualMotorWhenNotBusy)
        {
            if(!isMotorBusy(MotorSide.LEFT))  ldp = 0.0;
            if(!isMotorBusy(MotorSide.RIGHT)) rdp = 0.0;
        }
        else if(!isBusy())
        {
            ldp = 0.0;
            rdp = 0.0;
        }

        move(ldp, rdp);

        double ct = ptmr.seconds();
        if (ct > nextPrintTime)
        {
            nextPrintTime = ct + printTimeout;
            RobotLog.ii("SJH", "%4d lpwr: %5.3f rpwr: %5.3f err: %5.3f str %5.3f rt %5.3f chdg %d thdg %d",
                    frame, curLpower, curRpower, err, steer, rt.seconds(), curHdg, thdg);
        }
    }

//    void makeCorrections(double pwr, Direction dir)
//    {
//        double ldp;
//        double rdp;
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
//                RobotLog.ii("SJH", %4d ldc: %6d rdc: %6d diff: %2d " +
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
        int ldc = Math.abs(curLpos);
        int rdc = Math.abs(curRpos);

        //convert LR count difference to angle
        return countsToAngle(rdc - ldc, VEH_WIDTH);
    }

    private int getGyroError(int tgtHdg)
    {
        int robotError;
        int gHdg = curHdg;
        robotError = tgtHdg - gHdg;
        while (robotError > 180)   robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    private double getSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void setInitValues()
    {
        initLpos = robot.leftMotor.getCurrentPosition();
        initRpos = robot.rightMotor.getCurrentPosition();
        initHdg = robot.getGyroFhdg();
        while (initHdg <= -180) initHdg += 360;
        while (initHdg >   180) initHdg -= 360;
        curLpos = initLpos;
        curRpos = initRpos;
        curHdg  = initHdg;
    }

    public void setCurValues()
    {
        curLpos = robot.leftMotor.getCurrentPosition();
        curRpos = robot.rightMotor.getCurrentPosition();
        curHdg  = robot.getGyroFhdg();
        if(robot.colorEnabled)
        {
            curRed = robot.colorSensor.red();
            curGrn = robot.colorSensor.green();
            curBlu = robot.colorSensor.blue();
        }
        else
        {
            curRed = 0;
            curGrn = 0;
            curBlu = 0;
        }
        estimatePosition();
    }

    public void logStartValues(String note)
    {
        if(logData)
        {
            dl.addField("BEGIN " + note);
            dl.addField(frame);
            dl.addField(initHdg);
            dl.addField(initLpos);
            dl.addField(initRpos);
            dl.addField(trgtHdg);
            dl.addField(trgtLpos);
            dl.addField(trgtRpos);
            dl.addField("");
            dl.newLine();
        }
        RobotLog.ii("SJH", "Begin ldc %6d rdc %6d Hdg %d",
                initLpos, initRpos, initHdg);
    }

    public void setEndValues(String note)
    {
        doneLpos = robot.leftMotor.getCurrentPosition();
        doneRpos = robot.rightMotor.getCurrentPosition();
        doneHdg  = robot.getGyroFhdg();
        logData(true, "DONE " + note);

        RobotLog.ii("SJH", "End ldc %6d rdc %6d Hdg %d",
                doneLpos, doneRpos, doneHdg);
    }

    public void logOverrun(double t)
    {
        setCurValues();
        logData(true, "LOGGING OVERRUN");
        int overCnt = 0;
        int goodCnt = 0;
        int overLpos = curLpos;
        int overRpos = curRpos;
        int overThresh = 3;
        ElapsedTime et = new ElapsedTime();
        while(op.opModeIsActive() && et.seconds() < t)
        {
            setCurValues();
            logData();

            if(Math.abs(curLpos - overLpos) <= overThresh &&
               Math.abs(curRpos - overRpos) <= overThresh)
            {
               goodCnt++;
            }
            else
            {
               goodCnt = 0;
               overLpos = curLpos;
               overRpos = curRpos;
            }

            if(overCnt >= 4 && goodCnt >=3)
            {
               break;
            }

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
            overCnt++;
        }
        logData(true, "ENDOVER");
    }

    boolean areMotorsStuck()
    {
        if(usePosStop)
        {
            double lp = Math.abs(curLpower);
            double rp = Math.abs(curRpower);

            if(accelTimer.seconds() < 0.5)
            {
                lposLast = curLpos;
                rposLast = curRpos;
                noMoveTimer.reset();
                return false;
            }

            int dLpos = Math.abs(lposLast - curLpos);
            int dRpos = Math.abs(rposLast - curRpos);

            //If power is above threshold and encoders aren't changing,
            //stop after noMoveTimeout
            if(noMoveTimer.seconds() > noMoveTimeout)
            {
                if ((lp >= 0.0 && dLpos < noMoveThreshLow) &&
                    (rp >= 0.0 && dRpos < noMoveThreshLow))
                {
                    RobotLog.ii("SJH", "MOTORS HAVE POWER BUT AREN'T MOVING - STOPPING %4.2f %4.2f",
                            lp, rp);
                    return true;
                }
                lposLast = curLpos;
                rposLast = curRpos;
                noMoveTimer.reset();
            }
        }
        return false;
    }

    boolean areDriveMotorsStuck()
    {
        if(usePosStop)
        {
            double lp = Math.abs(curLpower);
            double rp = Math.abs(curRpower);

            if(accelTimer.seconds() < 0.5)
            {
                lposLast = curLpos;
                rposLast = curRpos;
                noMoveTimer.reset();
                return false;
            }

            int dLpos = Math.abs(lposLast - curLpos);
            int dRpos = Math.abs(rposLast - curRpos);

            //If power is above threshold and encoders aren't changing,
            //stop after noMoveTimeout
            if(noMoveTimer.seconds() > noDriveMoveTimeout)
            {
                if ((lp >= 0.0 && dLpos < noMoveThreshLow) &&
                    (rp >= 0.0 && dRpos < noMoveThreshLow))
                {
                    RobotLog.ii("SJH", "DRIVE MOTORS HAVE POWER BUT AREN'T MOVING - STOPPING %4.2f %4.2f",
                            lp, rp);
                    return true;
                }
                if ((lp >= noMovePwrHi && dLpos < noMoveThreshHi) ||
                    (rp >= noMovePwrHi && dRpos < noMoveThreshHi))
                {
                    RobotLog.ii("SJH", "DRIVE MOTORS HAVE HI POWER BUT AREN'T MOVING - STOPPING %4.2f %4.2f",
                            lp, rp);
                    return true;
                }
                lposLast = curLpos;
                rposLast = curRpos;
                noMoveTimer.reset();
            }
        }
        return false;
    }

    boolean isMotorBusy(MotorSide side)
    {
        double btime = busyTimer.milliseconds();
        boolean motorBusy;
        if(side == MotorSide.LEFT)
        {
            motorBusy = Math.abs(trgtLpos - curLpos) > BUSYTHRESH;
            if(motorBusy) lBusyTime = btime + busyTimeOut;
            else if(btime < lBusyTime) motorBusy = true;
        }
        else
        {
            motorBusy = Math.abs(trgtRpos - curRpos) > BUSYTHRESH;
            if(motorBusy) rBusyTime = btime + busyTimeOut;
            else if(btime < rBusyTime) motorBusy = true;
        }

        return motorBusy;
    }

    boolean isBusy(int thresh)
    {
        double ct = ptmr.seconds();
        if(ct > nextBusyPrintTime)
        {
            nextBusyPrintTime = ct + printTimeout;
            RobotLog.ii("SJH", "ldc %6d rdc %6d  mptimer: %4.2f chdg %5d",
                    curLpos, curRpos, noMoveTimer.seconds(), curHdg);
        }

        BUSYTHRESH = thresh;

        boolean lBusy = isMotorBusy(MotorSide.LEFT);
        boolean rBusy = isMotorBusy(MotorSide.RIGHT);
        boolean busy = false;

        if(busyAnd)
        {
            busy = lBusy && rBusy; //true if both are busy
        }
        else
        {
            busy = lBusy || rBusy; //true if 1 is busy
        }
        return busy;
    }

    boolean isBusy()
    {
       return isBusy(DEF_BUSYTHRESH);
    }

    public void setBusyAnd(boolean busyAnd)
    {
        this.busyAnd = busyAnd;
    }

    public void setRampUp(boolean rampUp)
    {
        this.rampUp = rampUp;
    }

    public void setRampDown(boolean rampDown)
    {
        this.rampDown = rampDown;
    }

    public void setGangMotors(boolean gangMotors)
    {
        this.gangMotors = gangMotors;
    }

    public void setStopIndividualMotorWhenNotBusy(boolean stopIndvid)
    {
        this.stopIndividualMotorWhenNotBusy = stopIndvid;
    }

    void setColGyroOffset( int offset)
    {
        this.colGyroOffset = offset;
    }

    void setLFirst(boolean lFirst)
    {
        this.lFirst = lFirst;
    }

    public void setDrvTuner(double dtnr)
    {
        CPI = ENCODER_CPR * GEAR_REDUC / (CIRCUMFERENCE * dtnr);
    }

    public void setLogOverrun(boolean lo)
    {
        this.logOverrun = lo;
    }

    public void setUseSpeedThreads(boolean ust) { this.useSpeedThreads = ust; }

    public void logData(boolean force, String title)
    {
        double ldt = logDataTimer.time();
        boolean expired = ldt > logTime;
        if(logData && (expired || force))
        {
            if(expired) logTime = ldt + logDataTimeout;
            if(force) logTime = 0;
            String comment = "";
            if(title != null) comment = title;
            dl.addField(comment);
            dl.addField(frame);
            if(robot.gyro != null) dl.addField(curHdg);
            else                   dl.addField("");
            dl.addField(curLpos);
            dl.addField(curRpos);
            dl.addField(curLpower);
            dl.addField(curRpower);
            dl.addField(curRed);
            dl.addField(curGrn);
            dl.addField(curBlu);
            dl.addField(estPos.getX());
            dl.addField(estPos.getY());
            dl.addField(Math.toDegrees(estHdg));
            dl.newLine();
        }
    }

    public void logData(boolean force)
    {
        logData(force, null);
    }

    public void logData()
    {
        logData(false);
    }

    enum MotorSide {LEFT, RIGHT}

    private static double DRV_TUNER = 1.00;
    private final static double TRN_TUNER = 1.0;
    private final static double TURN_TOLERANCE = 2.0;
    private int colGyroOffset = 0;

    private final static double VEH_WIDTH   = ShelbyBot.BOT_WIDTH * TRN_TUNER;
    private static double WHL_DIAMETER = 4.1875; //Diameter of the wheel (inches)
    private final static int    ENCODER_CPR = ShelbyBot.ENCODER_CPR;
    private final static double GEAR_REDUC = 0.5;                   //Gear ratio

    private static double CIRCUMFERENCE = Math.PI * WHL_DIAMETER;
    private static double CPI = ENCODER_CPR *GEAR_REDUC / (CIRCUMFERENCE * DRV_TUNER);
    private static final double DEF_CPI = CPI;

    private static final double Kp_GyrCorrection = 0.008;
    private static final double Kp_EncCorrection = 0.01;
    public  static       double Kp_GyroTurn      = 0.04;
    private static final double Kd_GyroTurn      = 0.001;
    private static final double THRESH = Math.toRadians(0.004);

    public enum Direction {FORWARD, REVERSE}

    private ShelbyBot robot;

    private Point2d currPt = new Point2d(0.0, 0.0);
    private double startHdg = 0.0;

    public int frame   = 0;
    //private LinearOpMode lom;
    private ElapsedTime period  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime rt = new ElapsedTime();
    private ElapsedTime ptmr = new ElapsedTime();

    private int lposLast;
    private int rposLast;

    public int initLpos;
    public int initRpos;
    public int curLpos;
    public int curRpos;
    public int trgtLpos;
    public int trgtRpos;
    private int doneLpos;
    private int doneRpos;
    private int overLpos;
    private int overRpos;

    private int initHdg;
    public int curHdg;
    private int trgtHdg;
    private int doneHdg;
    private int overHdg;

    public int curRed = 0;
    public int curGrn = 0;
    public int curBlu = 0;

    private double initLpower;
    private double initRpower;
    private double curLpower;
    private double curRpower;

    private ShelbyBot.DriveDir lastDriveDir;
    private ShelbyBot.DriveDir curDriveDir;

    private double xPos = 0.0;
    private double yPos = 0.0;
    public Point2d estPos = new Point2d(xPos, yPos);
    private double  estHdg = 0.0;
    private int lastLcnt = 0;
    private int lastRcnt = 0;
    private int numPts = 0;

    private double noMoveTimeout = 1.0;
    private double noDriveMoveTimeout = 0.25;
    private int noMoveThreshLow = 4;
    private int noMoveThreshHi = 20;
    private double noMovePwrHi = 0.15;
    private ElapsedTime accelTimer  = new ElapsedTime();
    private ElapsedTime noMoveTimer = new ElapsedTime();

    private double printTimeout = 0.05;

    private double minSpeed = 0.08;
    private double minGyroTurnSpeed = 0.10;

    private LinearOpMode op = null;

    private boolean usePosStop = true;
    private boolean doStopAndReset = false;

    private double lastGyroError = 0;
    private boolean useDterm = false;
    private boolean gyroFirstGood = false;
    private ElapsedTime gyroGoodTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double gyroTimeout = 60;
    private int gyroGoodCount = 0;

    private ElapsedTime gyroFrameTime = new ElapsedTime();
    private ElapsedTime datalogtimer = new ElapsedTime();
    private DataLogger dl;
    private boolean logData = true;
    private double logDataTimeout = 5;
    public ElapsedTime logDataTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double logTime = logDataTimer.seconds();

    private int curCntTgt = 0;

    double nextPrintTime = ptmr.seconds();
    double nextBusyPrintTime = ptmr.seconds();

    private boolean logOverrun = true;
    private double overtime = 0.12;

    private double reducePower = 0.3;
    private double reduceTurnPower = 0.2;

    private boolean busyAnd = false;
    private boolean rampUp = true;
    private boolean rampDown = true;
    private boolean stopIndividualMotorWhenNotBusy = false;
    private boolean gangMotors = false;

    private int tickRate = 10;
    private static final int DEF_BUSYTHRESH = 16;
    public  static final int TURN_BUSYTHRESH = 10;
    private static int BUSYTHRESH = DEF_BUSYTHRESH;

    private ElapsedTime busyTimer = new ElapsedTime();
    private double busyTimeOut = 20;
    private double lBusyTime = 0;
    private double rBusyTime = 0;

    private boolean lFirst = true;

    private double turnTimeLimit = 5;

    class SpeedSetTask implements Runnable
    {
        public void run()
        {
            if(op.opModeIsActive())
            {
                if(speed != oldSpeed)
                {
                    oldSpeed = speed;
                    if(motor != null) motor.setPower(speed);
                }
            }

            try
            {
                Thread.sleep(0, 100);
            }
            catch (InterruptedException ie)
            {
                System.out.println("Interrupted");
            }
        }

        public void setMotor(DcMotor motor) {this.motor = motor;}
        public void setSpeed(double speed)  {this.speed = speed;}
        private DcMotor motor = null;
        private double speed    = 0.0;
        private double oldSpeed = -2.0;
    }

    private SpeedSetTask lSpdTask = new SpeedSetTask();
    private SpeedSetTask rSpdTask = new SpeedSetTask();
    private boolean useSpeedThreads = true;
    private Thread lftSpdThread = null;
    private Thread rgtSpdThread = null;
}


