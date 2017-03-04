package org.firstinspires.ftc.teamcode;

import android.widget.TextView;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.Date;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

@SuppressWarnings({"unused", "ForLoopReplaceableByForEach"})
@Autonomous(name="AutonShelby", group="Auton")
//@Disabled
public class FtcAutoShelby extends OpenCvCameraOpMode implements FtcMenu.MenuButtons
{
    public FtcAutoShelby()
    {
        super();
    }

    public static HalDashboard getDashboard()
    {
        return dashboard;
    }

    public void initRobot()
    {
        telemetry.addData("_","PLEASE WAIT - STARTING");
        telemetry.update();
        dashboard = HalDashboard.createInstance(telemetry);

        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);
        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));
        setup();
    }

    public void startMode()
    {
        dashboard.clearDisplay();
        do_main_loop();
    }

    @Override
    public void runOpMode()
    {
        initRobot();
//        while(!isStarted() && !isStopRequested())
//        {
//            if(robot.gyro != null)
//            {
//                int chdg = robot.gyro.getIntegratedZValue();
//                dashboard.displayPrintf(6, "GHDG: %d", chdg);
//                DbgLog.msg("SJH INIT CHDG %d", chdg);
//            }
//            sleep(10);
//        }
        waitForStart();
        startMode();
        stopMode();
    }

    public void runPeriodic(double elapsedTime)
    {
    }

    public void stopMode()
    {
        cleanupCamera();
        if(drvTrn != null) drvTrn.stopMotion();
        dl.closeDataLogger();
    }

    private void setup()
    {
        dashboard.displayPrintf(2, "STATE: %s", "INITIALIZING - PLEASE WAIT FOR MENU");
        DbgLog.msg("SJH: SETUP");
        hardwareMap.logDevices();

        robot.init(this);

        drvTrn.init(robot);
        drvTrn.setOpMode(this);

        setupLogger();
        drvTrn.setDataLogger(dl);

        initOpenCv();

        imgProc = new BeaconDetector();
        bd = (BeaconFinder) imgProc;

        doMenus();

        Points pts = new Points(startPos, alliance, beaconChoice, parkChoice, useFly2Light);
        pathSegs = pts.getSegments();

        initHdg = (int)(Math.round(pathSegs[0].getFieldHeading()));

        ShelbyBot.DriveDir startDdir = pathSegs[0].getDir();
        robot.setDriveDir(startDdir);

        dashboard.displayPrintf(0, "GYRO CALIBRATING DO NOT TOUCH OR START");

        if (robot.leftMotor  != null &&
            robot.rightMotor != null &&
            robot.gyro       != null)
        {
            int lms = robot.leftMotor.getMaxSpeed();
            int rms = robot.rightMotor.getMaxSpeed();
            DbgLog.msg("SJH: MaxSpeeds %d %d", lms, rms);

            gyroReady = robot.calibrateGyro();
        }

        if(gyroReady)
            dashboard.displayPrintf(0, "GYRO CALIBATED!!");

        robot.lpusher.setPosition(L_DN_PUSH_POS);
        robot.rpusher.setPosition(R_DN_PUSH_POS);

        if(team == Team.SNOWMAN)
        {
            DEF_SHT_PWR = 0.75;
        }

        DbgLog.msg("SJH ROUTE: \n" + pts.toString());

        Point2d currPoint = pathSegs[0].getStrtPt();
        drvTrn.setCurrPt(currPoint);

        if(startPos == Field.StartPos.START_R_PUSHER && robot.gyroReady)
        {
            dashboard.displayPrintf(1, "PERFORM START POS MOVE NOW TO HDG %d", initHdg);
            robot.gyro.resetZAxisIntegrator();

            ElapsedTime botRotTimer = new ElapsedTime();
            while (botRotTimer.seconds() < 10)
            {
                int chdg = robot.gyro.getIntegratedZValue();
                dashboard.displayPrintf(0, "GHDG: %d", chdg);
                DbgLog.msg("SJH RMOVE CHDG %d", chdg);
                sleep(10);
            }

            dashboard.displayPrintf(1, "NO MORE MOVEMENT");
        }

        drvTrn.setStartHdg(initHdg);
        robot.setInitHdg(initHdg);

        DbgLog.msg("SJH Start %s.", currPoint);
        dashboard.displayPrintf(3, "PATH: Start at %s", currPoint);

        DbgLog.msg("SJH IHDG %4d", initHdg);

        imgProc.setTelemetry(telemetry);
    }

    private void do_main_loop()
    {
        timer.reset();
        startTimer.reset();
        dl.resetTime();

        DbgLog.msg("SJH: STARTING AT %4.2f", timer.seconds());
        if(logData)
        {
            Point2d spt = pathSegs[0].getStrtPt();
            dl.addField("START");
            dl.addField(initHdg);
            dl.addField(spt.getX());
            dl.addField(spt.getY());
            dl.newLine();
        }

        DbgLog.msg("SJH: Delaying for %4.2f seconds", delay);
        ElapsedTime delayTimer = new ElapsedTime();
        while (opModeIsActive() && delayTimer.seconds() < delay)
        {
            idle();
        }

        DbgLog.msg("SJH: Done delay");

        DbgLog.msg("SJH START CHDG %d", robot.gyro.getIntegratedZValue());

        robot.gyro.resetZAxisIntegrator();

        boolean SkipNextSegment = false;
        for (int i = 0; i < pathSegs.length; ++i)
        {
            if(!opModeIsActive() || isStopRequested()) break;

            String segName = pathSegs[i].getName();
            DbgLog.msg("SJH: Starting segment %s at %4.2f", segName,
                    startTimer.seconds());

            //noinspection ConstantConditions
            if (SkipNextSegment)
            {
                SkipNextSegment = false;
                DbgLog.msg("SJH: Skipping segment %s", pathSegs[i].getName());
                if(i < pathSegs.length - 1)
                {
                    DbgLog.msg("SJH: Setting segment %s start pt to %s",
                            pathSegs[i+1].getName(),
                            pathSegs[i].getStrtPt());
                    pathSegs[i+1].setStrtPt(pathSegs[i].getStrtPt());
                }
                continue;
            }

            Segment curSeg;

            if(curPos == null || !useImageLoc)
            {
                curSeg = pathSegs[i];
            }
            else
            {
                drvTrn.setCurrPt(curPos);
                curSeg = new Segment("CURSEG", curPos, pathSegs[i].getTgtPt());
            }
            curPos = null;

            if(curSeg.getStrtPt().getX() == curSeg.getTgtPt().getX() &&
               curSeg.getStrtPt().getY() == curSeg.getTgtPt().getY())
            {
                continue;
            }

            robot.setDriveDir(curSeg.getDir());

            drvTrn.setInitValues();
            drvTrn.logData(true, segName + " \'" +
                                 curSeg.getStrtPt().toString() + "\' - \'" +
                                 curSeg.getTgtPt().toString()  + "\' H:" +
                                 curSeg.getFieldHeading());

            drvTrn.logData(true, segName + " encoderTurn");
            DbgLog.msg("SJH: ENCODER TURN %s", curSeg.getName());
            doEncoderTurn(curSeg.getFieldHeading()); //quick but rough

            //drvTrn.setInitValues();
            //drvTrn.logData(true, segName + " gyroTurn");
            //DbgLog.msg("SJH: GYRO TURN %s", curSeg.getName());
            //doGyroTurn(curSeg.getFieldHeading()); //fine tune using gyro

            drvTrn.setInitValues();
            DbgLog.msg("SJH: Setting drive tuner to %4.2f", curSeg.getDrvTuner());
            drvTrn.logData(true, segName + " move");
            drvTrn.setDrvTuner(curSeg.getDrvTuner());
            doMove(curSeg);

            Double pturn = curSeg.getPostTurn();

            if(usePostTurn && pturn != null)
            {
                drvTrn.setInitValues();
                DbgLog.msg("SJH ENCODER POST TURN %s", curSeg.getName());
                drvTrn.logData(true, segName + " postEncoderTurn");
                doEncoderTurn(pturn);

//                drvTrn.setInitValues();
//                DbgLog.msg("SJH: GRYO POST TURN %s", curSeg.getName());
//                drvTrn.logData(true, segName + " postGyroTurn");
//                doGyroTurn(pturn);
            }

            if(!opModeIsActive() || isStopRequested())
            {
                drvTrn.stopMotion();
                break;
            }

            DbgLog.msg("SJH Planned pos: %s %s",
                    pathSegs[i].getTgtPt(),
                    pathSegs[i].getFieldHeading());


            Segment.Action act = curSeg.getAction();

            if(act != Segment.Action.NOTHING)
            {
                drvTrn.setInitValues();
                drvTrn.logData(true, segName + " action " + act.toString());
            }

            switch (act)
            {
                case SHOOT:
                    do_shoot();
                    break;

                case FIND_BEACON:
                    do_findAndPushBeacon(curSeg.getTgtPt());

                    if(robot.dim != null)
                    {
                        robot.dim.setLED(0, false); robot.dim.setLED(1, false);
                        robot.dim.setLED(0, false); robot.dim.setLED(0, false);
                    }

                    break;

                case RST_PUSHER:
                    robot.lpusher.setPosition(L_DN_PUSH_POS);
                    break;

                case NOTHING:
                    break;
            }
        }
    }

    private void doMove(Segment seg)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        drvTrn.setBusyAnd(false);
        String  snm = seg.getName();
        Point2d spt = seg.getStrtPt();
        Point2d ept = seg.getTgtPt();
        double  fhd = seg.getFieldHeading();
        ShelbyBot.DriveDir dir = seg.getDir();
        double speed = seg.getSpeed();
        double fudge = seg.getDrvTuner();
        Segment.TargetType ttype = seg.getTgtType();

        RobotLog.ii("SJH", "Drive %s %s %s %6.2f %3.2f %s tune: %4.2f %s",
                snm, spt, ept, fhd, speed, dir, fudge, ttype);

        dashboard.displayPrintf(2, "STATE: %s %s %s - %s %6.2f %3.2f %s",
                "DRIVE", snm, spt, ept, fhd, speed, dir);

        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;

        timer.reset();

        if(robot.colorSensor != null && seg.getTgtType() == Segment.TargetType.COLOR)
        {
            double pct = 0.94;
            Point2d cept = new Point2d(pct * (ept.getX() - spt.getX()) + spt.getX(),
                                       pct * (ept.getY() - spt.getY()) + spt.getY());

            int targetHdg = (int) Math.round(fhd);
            drvTrn.driveToPointLinear(cept, speed, ddir, targetHdg);

            DbgLog.msg("SJH: Turning on colorSensor LED");
            robot.turnColorOn();

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int colDist = drvTrn.distanceToCounts(cept.distance(ept));
            int segOver = drvTrn.distanceToCounts(2.0);

            drvTrn.setInitValues();

            sleep(10);

            double colSpd = 0.15;
            DbgLog.msg("SJH: Color Driving to pt %s at speed %4.2f", ept, colSpd);
            drvTrn.logData(true, "FIND_LINE CDIST: " + colDist);
            drvTrn.moveInit(colSpd, colSpd);

            while(opModeIsActive() && !isStopRequested())
            {
                drvTrn.setCurValues();
                drvTrn.estimatePosition();
                drvTrn.logData();

                int lTrav = drvTrn.curLpos  - drvTrn.initLpos;
                int rTrav = drvTrn.curRpos  - drvTrn.initRpos;

                int totColor = drvTrn.curRed + drvTrn.curGrn + drvTrn.curBlu;
                if (totColor > COLOR_THRESH)
                {
                    drvTrn.setEndValues("COLOR_FIND");
                    drvTrn.stopMotion();
                    DbgLog.msg("SJH: FOUND LINE");
                    robot.turnColorOff();
                    drvTrn.setCurrPt(ept);
                    break;
                }
                else if(Math.abs(lTrav) > (colDist + segOver) ||
                        Math.abs(rTrav) > (colDist + segOver))
                {
                    drvTrn.setEndValues("COLOR_MISS");
                    drvTrn.stopMotion();
                    DbgLog.msg("SJH: REACHED OVERRUN PT - Backing up a bit");
                    robot.turnColorOff();
                    double rDst = drvTrn.countsToDistance(segOver);
                    drvTrn.driveDistanceLinear(rDst, 0.2, Drivetrain.Direction.REVERSE);
                    drvTrn.setCurrPt(ept);
                    break;
                }

                robot.waitForTick(10);
            }

            sleep(postSleep);
        }
        else
        {
            int targetHdg = (int) Math.round(seg.getFieldHeading());
            drvTrn.driveToPointLinear(ept, speed, ddir, targetHdg);
        }

        RobotLog.ii("SJH", "Completed move %s. Time: %6.3f HDG: %4d",
                seg.getName(), timer.time(), robot.getGyroFhdg());
        sleep(postSleep);
    }


    private void doEncoderTurn(double fHdg)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        drvTrn.setBusyAnd(true);
        int cHdg = drvTrn.curHdg;
        int tHdg = (int) Math.round(fHdg);
        double angle = tHdg - cHdg;
        DbgLog.msg("SJH: doEncoderTurn CHDG %4d THDG %4d", cHdg, tHdg);

        while (angle <= -180.0) angle += 360.0;
        while (angle >   180.0) angle -= 360.0;
        if(Math.abs(angle) <= 2.0) return;

        DbgLog.msg("SJH: Turn %5.2f", angle);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        drvTrn.ctrTurnLinear(angle, DEF_ENCTRN_PWR);
        cHdg = robot.getGyroFhdg();
        DbgLog.msg("SJH Completed turn %5.2f. Time: %6.3f CHDG: %4d",
                angle, timer.time(), cHdg);
        sleep(postSleep);
    }

    private void doGyroTurn(double fHdg)
    {
        if(!gyroReady) return;
        if(!opModeIsActive() || isStopRequested()) return;
        int cHdg = drvTrn.curHdg;
        int tHdg = (int) Math.round(fHdg);

        DbgLog.msg("SJH: doGyroTurn CHDG %4d THDG %4d", cHdg, tHdg);

        if(Math.abs(tHdg-cHdg) <= 1.0)
            return;

        timer.reset();
        drvTrn.ctrTurnToHeading(tHdg, DEF_GYRTRN_PWR);

        cHdg = drvTrn.curHdg;
        DbgLog.msg("SJH Completed turnGyro %4d. Time: %6.3f CHDG: %4d",
                tHdg, timer.time(), cHdg);
        sleep(postSleep);
    }

    private BeaconFinder.BeaconSide findPushSide(BeaconFinder.BeaconSide bSide,
                                                 BeaconFinder.BeaconSide rSide)
    {
        int blue_led = 0;
        int red_led  = 1;

        BeaconFinder.BeaconSide pushSide = BeaconFinder.BeaconSide.UNKNOWN;

        if (bSide == rSide) return pushSide;

        if      ( alliance == Field.Alliance.BLUE ) pushSide = bSide;
        else if ( alliance == Field.Alliance.RED  ) pushSide = rSide;

        if(robot.dim != null)
        {
            if(pushSide == bSide)
            {
                robot.dim.setLED(blue_led, true);  robot.dim.setLED(red_led, false);
            }
            else if(pushSide == rSide)
            {
                robot.dim.setLED(blue_led, false); robot.dim.setLED(red_led, true);
            }
            else
            {
                robot.dim.setLED(blue_led, false); robot.dim.setLED(red_led, false);
            }
        }

        DbgLog.msg("SJH: BEACON BSIDE %s RSIDE %s PSIDE %s", bSide, rSide, pushSide);
        return pushSide;
    }

    private void setPusher(BeaconFinder.BeaconSide pushSide)
    {
        dashboard.displayPrintf(5, "BUTTON: %s", pushSide);
        if (pushSide == BeaconFinder.BeaconSide.LEFT)
        {
            robot.lpusher.setPosition(L_UP_PUSH_POS);
            robot.rpusher.setPosition(R_DN_PUSH_POS);
            DbgLog.msg("SJH: Pushing left button");
        }
        else if (pushSide == BeaconFinder.BeaconSide.RIGHT)
        {
            robot.rpusher.setPosition(R_UP_PUSH_POS);
            robot.lpusher.setPosition(L_DN_PUSH_POS);
            DbgLog.msg("SJH: Pushing right button");
        }
        else
        {
            robot.lpusher.setPosition(L_DN_PUSH_POS);
            robot.rpusher.setPosition(R_DN_PUSH_POS);
            DbgLog.msg("SJH: Not Pushing A Button");
        }
    }

    private boolean do_findAndPushBeacon(boolean push, Segment seg)
    {
        DbgLog.msg("SJH: FIND BEACON ORDER!!!");
        dashboard.displayPrintf(2, "STATE: %s", "BEACON FIND");
        int timeout = 2000;
        BeaconFinder.BeaconSide blueSide = BeaconFinder.BeaconSide.UNKNOWN;
        BeaconFinder.BeaconSide redSide  = BeaconFinder.BeaconSide.UNKNOWN;
        BeaconFinder.BeaconSide pushSide = BeaconFinder.BeaconSide.UNKNOWN;

        imgProc.startSensing();
        sleep( 50 );

        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opModeIsActive()                              &&
               pushSide == BeaconFinder.BeaconSide.UNKNOWN   &&
               itimer.milliseconds() < timeout)
        {
            blueSide = bd.getBluePosSide();
            redSide  = bd.getRedPosSide();
            pushSide = findPushSide(blueSide, redSide);
            robot.waitForTick(20);
        }

        imgProc.stopSensing();

        setPusher(pushSide);
        if(push && pushSide != BeaconFinder.BeaconSide.UNKNOWN)
        {
            double tgtDist = 10.5;
            Point2d touchStart = seg.getTgtPt();
            double touchX = touchStart.getX();
            double touchY = touchStart.getY();

            int targetHdg;
            if (alliance == Field.Alliance.RED)
            {
                touchX -= tgtDist;
                targetHdg = 180;
            }
            else
            {
                touchY += tgtDist;
                targetHdg = 90;
            }

            Point2d touchEnd = new Point2d("TCHEND", touchX, touchY);

            drvTrn.logData(true, "PUSHING " + pushSide.toString());
            int dcount;
            robot.setDriveDir(ShelbyBot.DriveDir.PUSHER);
            //noinspection ConstantConditions
            dcount = drvTrn.driveToPointLinear(touchEnd, 0.2,
                    Drivetrain.Direction.FORWARD, targetHdg);

            double actDist = drvTrn.countsToDistance(dcount);

            double actTouchX = touchStart.getX();
            double actTouchY = touchStart.getY();

            if (alliance == Field.Alliance.RED)
            {
                actTouchX -= tgtDist;
                targetHdg = 0;
            }
            else
            {
                actTouchY += tgtDist;
                targetHdg = -90;
            }
            robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
            Point2d actTouchPt = new Point2d(actTouchX, actTouchY);
            drvTrn.logData(true, "AT PUSH " + actTouchPt.toString());
            drvTrn.setCurrPt(actTouchPt);
            drvTrn.driveToPointLinear(touchStart, 0.2,
                    Drivetrain.Direction.FORWARD, targetHdg);
            drvTrn.setCurrPt(touchStart);
            drvTrn.logData(true, "PUSHED?");
        }

        setPusher(BeaconFinder.BeaconSide.UNKNOWN);

        return pushSide != BeaconFinder.BeaconSide.UNKNOWN;
    }

    private void do_findAndPushBeacon(Point2d endPt)
    {
        DbgLog.msg("SJH: /BEACON/ > THE HUNT FOR BEACON");

        String beaconStep = "FIND";
        String driveStep = "INIT";

        BeaconFinder.BeaconSide blueSide = BeaconFinder.BeaconSide.UNKNOWN;
        BeaconFinder.BeaconSide redSide  = BeaconFinder.BeaconSide.UNKNOWN;
        BeaconFinder.BeaconSide pushSide = BeaconFinder.BeaconSide.UNKNOWN;

        boolean allDone = false, skipDrive = false;
        double baseSpeed = 0.2;
        double bConf = 0, zPos = 0, xPos = 0, nPos = 0, rDv = 0, lDv = 0;
        double tPow = 0, nAng = 0;
        double cHdg, hErr, nOff, mDir = 1;
        double curDistCount = 0.0;
        double curDist = 0.0;

        imgProc.startSensing();

        robot.setDriveDir(ShelbyBot.DriveDir.PUSHER);

        drvTrn.stopAndReset();
        drvTrn.setInitValues();

        sleep( 50 );

        double desHdg = 180;
        if (alliance == Field.Alliance.BLUE) desHdg = 90;

        drvTrn.logData(true, "push encoderTurn");
        doEncoderTurn(desHdg);
        drvTrn.ctrTurnToHeading( desHdg, baseSpeed );

        while( opModeIsActive() && !allDone ) {

            drvTrn.setCurValues();
            drvTrn.estimatePosition();
            drvTrn.logData(false, "BS:" + beaconStep + " DS:" + driveStep);

            cHdg = drvTrn.curHdg;
            if(cHdg < 0) cHdg += 360.0;
            //Determine which direction should go with sign of hErr
            hErr = desHdg - cHdg;

            curDistCount = (drvTrn.curLpos - drvTrn.initLpos +
                            drvTrn.curRpos - drvTrn.initRpos) / 2.0;
            curDist = drvTrn.countsToDistance(curDistCount);

            switch ( beaconStep )
            {
                case "FIND":

                    imgProc.logDebug();

                    if ( bd.getBeaconConf() > 0.25 )
                    {
                        beaconStep = "FORWARD";
                        DbgLog.msg("SJH: /BEACON/FIND > CONFIDENCE HIGH, MOVE ALONG" );
                    }
                    else
                    {
                        beaconStep = "LEAVE";
                        DbgLog.msg("SJH: /BEACON/FIND > CONFIDENCE LOW, NEXT STEP");
                    }

                    break;

                case "FORWARD":

                    imgProc.logDebug();
                    imgProc.logTelemetry();

                    blueSide = bd.getBluePosSide();
                    redSide  = bd.getRedPosSide();

                    bConf = bd.getBeaconConf();
                    //zPos  = bd.getBeaconPosZ();

                    switch ( driveStep ) {
                        case "TRY2":
                            DbgLog.msg("SJH: /BEACON/TRY2" );
                            mDir = -1;

                        case "INIT":

                            zPos = bd.getBeaconPosZ();
                            xPos = bd.getBeaconPosX();
                            drvTrn.logData(true, "INIT X:" + xPos + " Z:" + zPos);

                            if ( zPos < 15.0 && mDir == 1 && Math.abs( xPos ) > 1.5 ) {
                                drvTrn.driveDistanceLinear(15.0 - zPos,
                                        baseSpeed, Drivetrain.Direction.REVERSE);
                                zPos = 15.0;
                                xPos = bd.getBeaconPosX();
                            }

                            nOff = zPos * Math.tan( Math.toRadians( hErr ) );
                            nPos = xPos + nOff;
                            nAng = Math.atan( nPos / 9.0 );
                            tPow = 2.0 * nAng / Math.PI;

                            dl.addField("INIT");
                            dl.addField(xPos);
                            dl.addField(zPos);
                            dl.addField(nOff);
                            dl.addField(nPos);
                            dl.addField(nAng);
                            dl.addField(tPow);
                            dl.newLine();

                            rDv = mDir * Range.clip( baseSpeed + mDir * tPow, -0.25, 0.25 );
                            lDv = mDir * Range.clip( baseSpeed - mDir * tPow, -0.25, 0.25 );

                            DbgLog.msg("SJH: /BEACON/INIT > nOff: %5.2f, nPos: %5.2f, nAng: %5.2f",
                                    nOff, nPos, nAng );

                            if ( Math.abs( xPos ) > 1.5 )
                                drvTrn.moveInit(lDv, rDv);
                            else
                                skipDrive = true;

                            driveStep = "CENTER";
                            break;

                        case "CENTER":

                            DbgLog.msg("SJH: /BEACON/CENTER > r: %5.2f, l: %5.2f, d: %5.2f, a: %5.2f"
                                    , rDv, lDv, curDist, hErr );

                            if (Math.abs(curDist) > zPos / 2.0 || skipDrive)
                            {
                                doEncoderTurn(desHdg);
                                doGyroTurn(desHdg);

                                drvTrn.stopMotion();

                                xPos = bd.getBeaconPosX();
                                if ( Math.abs( xPos ) < 1.5 || mDir == -1 )
                                    driveStep = "READY";
                                else
                                    driveStep = "TRY2";
                            }

                            break;
                    }

                    // Good when the beacon is in view enough or at least
                    // some driving done.
                    if ( pushSide == BeaconFinder.BeaconSide.UNKNOWN &&
                         blueSide != BeaconFinder.BeaconSide.UNKNOWN &&
                         redSide != BeaconFinder.BeaconSide.UNKNOWN &&
                         driveStep.equals("CENTER") )
                    {
                        pushSide = findPushSide(blueSide, redSide);
                        DbgLog.msg("SJH: /BEACON/FORWARD > FOUND %s ON %s", alliance, pushSide );
                    }

                    setPusher(pushSide);

                    if ( driveStep.equals("READY") )
                    {
                        if ( pushSide == BeaconFinder.BeaconSide.UNKNOWN )
                        {
                            beaconStep = "BACKUP";
                            DbgLog.msg("SJH: /BEACON/FORWARD > NO SIDE DETECTED, EXITING AT %5.2f ",
                                    curDistCount );
                        }
                        else
                        {
                            beaconStep = "PUSH";
                            DbgLog.msg("SJH: /BEACON/FORWARD > TIME TO PUSH BUTTON ON THE %s",
                                    pushSide );
                        }
                    }

                    break;

                case "PUSH":

                    DbgLog.msg("SJH: /BEACON/PUSH > GOING TO PUSH BUTTON" );

                    drvTrn.logData(false, "PUSH " + pushSide.toString());
                    int c = drvTrn.driveDistanceLinear(18.0, baseSpeed,
                                                       Drivetrain.Direction.FORWARD,
                                                       (int)desHdg);

                    DbgLog.msg("SJH: /BEACON/PUSH > PUSHED THE BUTTON" );
                    beaconStep = "BACKUP";

                    break;

                case "BACKUP":

                    DbgLog.msg("SJH: /BEACON/BACKUP > BACKING UP %4.1f", curDistCount );

                    int lDcnt = drvTrn.curLpos - drvTrn.initLpos;
                    int rDcnt = drvTrn.curRpos - drvTrn.initRpos;
                    double backDist = drvTrn.countsToDistance((lDcnt + rDcnt) / 2.0);
                    backDist = Math.abs(backDist);

                    drvTrn.logData(false, "BACKUP " + backDist);
                    drvTrn.driveDistanceLinear(backDist, baseSpeed,
                            Drivetrain.Direction.REVERSE, (int)desHdg);
                    drvTrn.setCurrPt(endPt);

                    DbgLog.msg("SJH: /BEACON/BACKUP > BACKED UP" );
                    robot.lpusher.setPosition(L_DN_PUSH_POS);
                    robot.rpusher.setPosition(R_DN_PUSH_POS);
                    beaconStep = "LEAVE";

                    break;

                case "LEAVE":

                    allDone = true;
                    drvTrn.logData(true, "LEAVE");
                    DbgLog.msg("SJH: /BEACON/LEAVE > MOVING ALONG" );
                    break;

            }

            robot.waitForTick( 10 );
        }

        imgProc.stopSensing();

        drvTrn.stopMotion();
        drvTrn.setEndValues("DONE FINDPUSH");

        DbgLog.msg("SJH: /BEACON/ > MISSION COMPLETE");
    }

    private void do_shoot()
    {
        DbgLog.msg("SJH: SHOOT!!!");
        dashboard.displayPrintf(2, "STATE: %s", "SHOOT");
        ElapsedTime stimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        robot.shotmotor1.setPower(DEF_SHT_PWR);
        robot.shotmotor2.setPower(DEF_SHT_PWR);
        sleep(500);
        robot.sweepMotor.setPower(-DEF_SWP_PWR);
        robot.elevMotor.setPower(-DEF_ELV_PWR);
        sleep(1500);
        robot.shotmotor1.setPower(0);
        robot.shotmotor2.setPower(0);
        robot.sweepMotor.setPower(0);
        robot.elevMotor.setPower(0);
    }

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up;} //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    } //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    } //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }  //isMenuBackButton

    private void doMenus()
    {
        //
        // Create the menus.
        //
        //FtcChoiceMenu strategyMenu = new FtcChoiceMenu("STRATEGY:", null, this);
        FtcChoiceMenu<Field.StartPos> startPosMenu =
                new FtcChoiceMenu<>("START:", null, this);
        FtcChoiceMenu<Field.BeaconChoice> pushMenu =
                new FtcChoiceMenu<>("PUSH:", startPosMenu, this);
        FtcChoiceMenu<Field.ParkChoice> parkMenu   =
                new FtcChoiceMenu<>("PARK:", pushMenu, this);
        FtcChoiceMenu<Field.Alliance> allianceMenu =
                new FtcChoiceMenu<>("ALLIANCE:", parkMenu, this);
        FtcChoiceMenu<Team> teamMenu               =
                new FtcChoiceMenu<>("TEAM:", allianceMenu, this);
        FtcChoiceMenu<Boolean> routeMenu           =
                new FtcChoiceMenu<>("FLY2LIGHT:", teamMenu, this);
        //FtcValueMenu powerMenu     = new FtcValueMenu("SHOOTPOWER:", teamMenu, this,
        //                                                    0.0, 1.0, 0.05, 0.55, "%5.2f");
        FtcValueMenu delayMenu     = new FtcValueMenu("DELAY:", routeMenu, this,
                0.0, 20.0, 1.0, 0.0, "%5.2f");

        startPosMenu.addChoice("Start_A", Field.StartPos.START_A_SWEEPER, pushMenu);
        startPosMenu.addChoice("Start_B", Field.StartPos.START_B_SWEEPER, pushMenu);
        startPosMenu.addChoice("Start_R", Field.StartPos.START_R_PUSHER,  pushMenu);
        pushMenu.addChoice("BOTH", Field.BeaconChoice.BOTH, parkMenu);
        pushMenu.addChoice("NEAR", Field.BeaconChoice.NEAR, parkMenu);
        pushMenu.addChoice("FAR", Field.BeaconChoice.FAR, parkMenu);
        pushMenu.addChoice("NONE", Field.BeaconChoice.NONE, parkMenu);

        parkMenu.addChoice("CORNER", Field.ParkChoice.CORNER_PARK, allianceMenu);
        parkMenu.addChoice("CENTER", Field.ParkChoice.CENTER_PARK, allianceMenu);
        parkMenu.addChoice("CORNER", Field.ParkChoice.CORNER_PARK, allianceMenu);
        parkMenu.addChoice("DEFEND", Field.ParkChoice.DEFEND_PARK, allianceMenu);

        allianceMenu.addChoice("RED",  Field.Alliance.RED, teamMenu);
        allianceMenu.addChoice("BLUE", Field.Alliance.BLUE, teamMenu);

        teamMenu.addChoice("Snowman", Team.SNOWMAN, routeMenu);
        teamMenu.addChoice("Sonic", Team.SONIC, routeMenu);

        routeMenu.addChoice("FALSE", Boolean.FALSE, delayMenu);
        routeMenu.addChoice("TRUE",  Boolean.TRUE,  delayMenu);

        //
        // Walk the menu tree starting with the strategy menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(startPosMenu, this);

        //
        // Set choices variables.
        //

        startPos = startPosMenu.getCurrentChoiceObject();
        beaconChoice = pushMenu.getCurrentChoiceObject();
        parkChoice = parkMenu.getCurrentChoiceObject();
        alliance = allianceMenu.getCurrentChoiceObject();
        team = teamMenu.getCurrentChoiceObject();
        useFly2Light = routeMenu.getCurrentChoiceObject();
        delay = delayMenu.getCurrentValue();

        int lnum = 3;
        dashboard.displayPrintf(lnum++, "START: %s", startPos);
        dashboard.displayPrintf(lnum++, "PUSH: %s", beaconChoice);
        dashboard.displayPrintf(lnum++, "PARK: %s", parkChoice);
        dashboard.displayPrintf(lnum++, "ALLIANCE: %s", alliance);
        dashboard.displayPrintf(lnum++, "TEAM: %s", team);
        dashboard.displayPrintf(lnum++, "FLY2LIGHT: %s", useFly2Light);

        DbgLog.msg("SJH: STARTPOS %s", startPos);
        DbgLog.msg("SJH: PUSH     %s", beaconChoice);
        DbgLog.msg("SJH: PARK     %s", parkChoice);
        DbgLog.msg("SJH: ALLIANCE %s", alliance);
        DbgLog.msg("SJH: TEAM     %s", team);
        DbgLog.msg("SJH: FLY2LIGHT: %s", useFly2Light);
        DbgLog.msg("SJH: DELAY    %4.2f", delay);
    }

    private void setupLogger()
    {
        if (logData)
        {
            Date day = new Date();
            dl = new DataLogger(day.toString() + "autonomousData");
            dl.addField("NOTE");
            dl.addField("FRAME");
            dl.addField("Gyro");
            dl.addField("LENC");
            dl.addField("RENC");
            dl.addField("LPWR");
            dl.addField("RPWR");
            dl.addField("RED");
            dl.addField("GRN");
            dl.addField("BLU");
            dl.addField("ESTX");
            dl.addField("ESTY");
            dl.addField("ESTH");
            dl.newLine();
        }
    }

    private enum Team
    {
        SONIC,
        SNOWMAN
    }

    private final static double L_DN_PUSH_POS = 1.0;
    private final static double R_DN_PUSH_POS = 0.0;
    private final static double L_UP_PUSH_POS = 0.0;
    private final static double R_UP_PUSH_POS = 1.0;

    private final static double DEF_ENCTRN_PWR  = 0.4;
    private final static double DEF_GYRTRN_PWR = 0.4;

    private final static double DEF_SWP_PWR = 1.0;
    private final static double DEF_ELV_PWR = 0.5;

    private Segment[] pathSegs;

    private ShelbyBot   robot = new ShelbyBot();
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime startTimer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private BeaconFinder bd;

    private static Point2d curPos;
    private static double  curHdg;
    private int initHdg = 0;
    private boolean gyroReady;
    private boolean usePostTurn = true;

    private static Field.StartPos startPos = Field.StartPos.START_A_SWEEPER;
    private static Field.BeaconChoice beaconChoice = Field.BeaconChoice.NEAR;
    private static Field.ParkChoice parkChoice = Field.ParkChoice.CENTER_PARK;
    private static Field.Alliance alliance = Field.Alliance.RED;
    private static Team team = Team.SNOWMAN;
    private static double DEF_SHT_PWR = 0.85;

    private static HalDashboard dashboard = null;

    private int RED_THRESH = 15;
    private int GRN_THRESH = 15;
    private int BLU_THRESH = 15;
    private int COLOR_THRESH = 20;

    private double delay = 0.0;

    private boolean useFly2Light = true;
    private boolean useImageLoc  = false;

    private boolean firstInState = true;

    private DataLogger dl;
    private boolean logData = true;

    private int postSleep = 150;
}

