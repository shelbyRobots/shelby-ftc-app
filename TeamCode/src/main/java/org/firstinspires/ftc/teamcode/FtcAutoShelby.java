package org.firstinspires.ftc.teamcode;

import android.widget.TextView;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.Calendar;
import java.util.Date;
import java.util.Locale;

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
        drvTrn.start();
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
//                RobotLog.dd("SJH", "INIT CHDG %d", chdg);
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
        if(drvTrn != null) drvTrn.cleanup();
        //for(int i = 1; i <= numBeacons; i++) imgProc.saveImage(i);
        cleanupCamera();
        dl.closeDataLogger();
        dashboard.clearDisplay();
    }

    private void setup()
    {
        dashboard.displayPrintf(2, "STATE: %s", "INITIALIZING - PLEASE WAIT FOR MENU");
        RobotLog.ii("SJH", "SETUP");
        hardwareMap.logDevices();

        robot.init(this);

        drvTrn.init(robot);
        drvTrn.setOpMode(this);
        drvTrn.setUseSpeedThreads(false);
        drvTrn.setRampUp(false);

        initOpenCv();

        imgProc = new BeaconDetector();
        bd = (BeaconFinder) imgProc;

        doMenus();

        setupLogger();
        drvTrn.setDataLogger(dl);

        dl.addField("Start: " + startPos.toString());
        dl.addField("Push: " + beaconChoice.toString());
        dl.addField("Park: " + parkChoice.toString());
        dl.addField("Alliance: " + alliance.toString());
        RobotLog.ii("SJH", "STARTPOS %s", startPos);
        RobotLog.ii("SJH", "PUSH     %s", beaconChoice);
        RobotLog.ii("SJH", "PARK     %s", parkChoice);
        RobotLog.ii("SJH", "ALLIANCE %s", alliance);
        RobotLog.ii("SJH", "DELAY    %4.2f", delay);

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
            gyroReady = robot.calibrateGyro();
        }

        if(gyroReady)
            dashboard.displayPrintf(0, "GYRO CALIBATED!!");

        robot.lpusher.setPosition(L_DN_PUSH_POS);
        robot.rpusher.setPosition(R_DN_PUSH_POS);

        if(alliance == Field.Alliance.BLUE) drvTrn.setLFirst(false);

        if(team == Team.SNOWMAN)
        {
            DEF_SHT_PWR = 0.75;
        }

        RobotLog.ii("SJH", "ROUTE: \n" + pts.toString());

        Point2d currPoint = pathSegs[0].getStrtPt();
        drvTrn.setCurrPt(currPoint);

        if(startPos == Field.StartPos.START_R_PUSHER && robot.gyroReady)
        {
            dashboard.displayPrintf(1, "PERFORM START POS MOVE NOW TO HDG %d", initHdg);
            robot.gyro.resetZAxisIntegrator();

            ElapsedTime botRotTimer = new ElapsedTime();
            while (botRotTimer.seconds() < 20)
            {
                int chdg = robot.gyro.getIntegratedZValue();
                dashboard.displayPrintf(0, "GHDG: %d", chdg);
                RobotLog.ii("SJH", "RMOVE CHDG %d", chdg);
                sleep(10);
            }

            dashboard.displayPrintf(1, "NO MORE MOVEMENT");
        }

        drvTrn.setStartHdg(initHdg);
        robot.setInitHdg(initHdg);

        RobotLog.ii("SJH", "Start %s.", currPoint);
        dashboard.displayPrintf(3, "PATH: Start at %s", currPoint);

        RobotLog.ii("SJH", "IHDG %4d", initHdg);

        imgProc.setTelemetry(telemetry);
    }

    private void do_main_loop()
    {
        timer.reset();
        startTimer.reset();
        dl.resetTime();

        RobotLog.ii("SJH", "STARTING AT %4.2f", timer.seconds());
        if(logData)
        {
            Point2d spt = pathSegs[0].getStrtPt();
            dl.addField("START");
            dl.addField(initHdg);
            dl.addField(spt.getX());
            dl.addField(spt.getY());
            dl.newLine();
        }

        RobotLog.ii("SJH", "Delaying for %4.2f seconds", delay);
        ElapsedTime delayTimer = new ElapsedTime();
        while (opModeIsActive() && delayTimer.seconds() < delay)
        {
            idle();
        }

        RobotLog.ii("SJH", "Done delay");

        RobotLog.ii("SJH", "START CHDG %d", robot.gyro.getIntegratedZValue());

        robot.gyro.resetZAxisIntegrator();

        boolean SkipNextSegment = false;
        for (int i = 0; i < pathSegs.length; ++i)
        {
            if(!opModeIsActive() || isStopRequested()) break;

            String segName = pathSegs[i].getName();
            RobotLog.ii("SJH", "Starting segment %s at %4.2f", segName,
                    startTimer.seconds());

            //noinspection ConstantConditions
            if (SkipNextSegment)
            {
                SkipNextSegment = false;
                RobotLog.ii("SJH", "Skipping segment %s", pathSegs[i].getName());
                if(i < pathSegs.length - 1)
                {
                    RobotLog.ii("SJH", "Setting segment %s start pt to %s",
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
            String segLogStr = String.format(Locale.US, "%s - %s H: %4.1f",
                    curSeg.getStrtPt().toString(),
                    curSeg.getTgtPt().toString(),
                    curSeg.getFieldHeading());
            drvTrn.logData(true, segName + " " + segLogStr);

            RobotLog.ii("SJH", "ENCODER TURN %s", curSeg.getName());
            doEncoderTurn(curSeg.getFieldHeading(), segName + " encoderTurn"); //quick but rough

            if(curSeg.getName().equals("BECN1") ||
               curSeg.getName().equals("BECN2"))
            {
                RobotLog.ii("SJH", "GYRO TURN %s", curSeg.getName());
                doGyroTurn(curSeg.getFieldHeading(), segName + " gyroTurn"); //fine tune using gyro
            }

            if (curSeg.getAction() == Segment.Action.SHOOT)
            {
                robot.shotmotor1.setPower(DEF_SHT_PWR);
                robot.shotmotor2.setPower(DEF_SHT_PWR);
                //robot.sweepMotor.setPower(-DEF_SWP_PWR * 0.1);
            }
            doMove(curSeg);

            Double pturn = curSeg.getPostTurn();
            if(usePostTurn && pturn != null)
            {
                RobotLog.ii("SJH", "ENCODER POST TURN %s", curSeg.getName());
                doEncoderTurn(pturn, segName + " postEncoderTurn");

//                RobotLog.ii("SJH", "GRYO POST TURN %s", curSeg.getName());
//                doGyroTurn(pturn, segName + " postGyroTurn");
            }

            if(!opModeIsActive() || isStopRequested())
            {
                drvTrn.stopMotion();
                break;
            }

            RobotLog.ii("SJH", "Planned pos: %s %s",
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
                    drvTrn.driveToTarget(0.13, 18);
                    break;

                case FIND_BEACON:
                    //do_findAndPushBeacon(curSeg.getTgtPt());
                    do_findAndPushBeacon(true, curSeg);

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

        drvTrn.setInitValues();
        RobotLog.ii("SJH", "Setting drive tuner to %4.2f", seg.getDrvTuner());
        drvTrn.logData(true, seg.getName() + " move");
        drvTrn.setDrvTuner(seg.getDrvTuner());

        drvTrn.setBusyAnd(true);
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

        boolean doCorrect = true;
        boolean singleSeg = true;
        if(robot.colorSensor != null && seg.getTgtType() == Segment.TargetType.COLOR)
        {
            colSegNum++;
            int colGyroOffset = 80;
            if(alliance == Field.Alliance.RED)
            {
                if(colSegNum == 1)
                {
                    colGyroOffset = 60;
                }
                else
                {
                    colGyroOffset = 80;
                }
            }
            else
            {
                if(colSegNum == 1)
                {
                    colGyroOffset = 120;
                }
                else
                {
                    colGyroOffset = 120;
                }
            }

            drvTrn.setColGyroOffset(colGyroOffset);
            drvTrn.setInitValues();
            double pct = 0.90;
            double fullSegLen = seg.getLength();
            int colSegLbeg = drvTrn.curLpos;
            int colSegRbeg = drvTrn.curRpos;
            int colSegLend = colSegLbeg + drvTrn.distanceToCounts(fullSegLen);
            int colSegRend = colSegRbeg + drvTrn.distanceToCounts(fullSegLen);

            int linLpos = colSegLend;
            int linRpos = colSegRend;

            Point2d cept = new Point2d(pct * (ept.getX() - spt.getX()) + spt.getX(),
                                       pct * (ept.getY() - spt.getY()) + spt.getY());

            int targetHdg = (int) Math.round(fhd);

            double colDist = cept.distance(ept);
            double ovrDist = 2.0;
            int colCnts = drvTrn.distanceToCounts(colDist);
            int ovrCnts = drvTrn.distanceToCounts(ovrDist);

            if(singleSeg)
            {
                drvTrn.driveDistanceLinear(fullSegLen, speed, ddir, targetHdg, true);
            }
            else
            {
                drvTrn.driveToPointLinear(cept, speed, ddir, targetHdg);
                //drvTrn.driveToTarget(0.2, 20);
                robot.turnColorOn();

                sleep(10);

                double colSpd = 0.10;
                RobotLog.ii("SJH", "Color Driving to pt %s at speed %4.2f", ept, colSpd);
                String colDistStr = String.format(Locale.US, "%4.2f %s",
                        colDist,
                        cept.toString());
                drvTrn.logData(true, "FIND_LINE CDIST: " + colDistStr);

                drvTrn.driveDistance(colDist+ovrDist, colSpd, Drivetrain.Direction.FORWARD);

                boolean foundLine = false;

                while(opModeIsActive() && !isStopRequested())
                {
                    drvTrn.setCurValues();
                    drvTrn.logData();

                    int lTrav = Math.abs(drvTrn.curLpos  - drvTrn.initLpos);
                    int rTrav = Math.abs(drvTrn.curRpos  - drvTrn.initRpos);

                    int totColor = drvTrn.curRed + drvTrn.curGrn + drvTrn.curBlu;

                    if (totColor > COLOR_THRESH)
                    {
                        linLpos = drvTrn.curLpos;
                        linRpos = drvTrn.curRpos;
                        colGyroOffset = 50;
                        if(snm.equals("BECN2"))
                        {
                            linLpos -= colGyroOffset;
                            linRpos -= colGyroOffset;
                        }
                        drvTrn.stopMotion();
                        drvTrn.setEndValues("COLOR_FIND " + linLpos + " " + linRpos);
                        RobotLog.ii("SJH", "FOUND LINE");
                        foundLine = true;
                        drvTrn.trgtLpos = linLpos;
                        drvTrn.trgtRpos = linRpos;
                        drvTrn.logOverrun(0.1);
                        break;
                    }

                    if(lTrav > (colCnts + ovrCnts) ||
                       rTrav > (colCnts + ovrCnts))
                    {
                        drvTrn.stopMotion();
                        drvTrn.setEndValues("COLOR_MISS - go to" + linLpos + " " + linRpos);
                        RobotLog.ii("SJH", "REACHED OVERRUN PT - Backing up a bit");
                        drvTrn.trgtLpos = linLpos;
                        drvTrn.trgtRpos = linRpos;
                        drvTrn.logOverrun(0.1);
                        break;
                    }

                    drvTrn.makeGyroCorrections(colSpd, targetHdg, Drivetrain.Direction.FORWARD);

                    drvTrn.frame++;
                    robot.waitForTick(10);
                }
            }

            robot.turnColorOff();
        }
        else
        {
            int targetHdg = (int) Math.round(seg.getFieldHeading());
            drvTrn.driveToPointLinear(ept, speed, ddir, targetHdg);
        }

        //If segment action is shoot, force bot to turn slightly to try to get balls to fall away from beacons

        int dtlCorrect = 18;
        if(seg.getAction() == Segment.Action.SHOOT)
        {
            dtlCorrect = 16;
            double shotAdjAngle = 0.0;
            int shootAdjCnt = drvTrn.angleToCounts(shotAdjAngle, ShelbyBot.BOT_WIDTH/2.0);
            String sAdjStr = String.format(Locale.US, "SHOOT ANGLE ADJ %d", shootAdjCnt);
            drvTrn.logData(true, sAdjStr);
            if(alliance == Field.Alliance.RED && startPos != Field.StartPos.START_R_PUSHER)
            {
                drvTrn.trgtLpos += shootAdjCnt;
                drvTrn.trgtRpos -= shootAdjCnt;
            }
            if(alliance == Field.Alliance.BLUE && startPos != Field.StartPos.START_R_PUSHER)
            {
                drvTrn.trgtLpos -= shootAdjCnt;
                drvTrn.trgtRpos += shootAdjCnt;
            }
        }

        if(snm.equals("PRECTR") ||
           snm.equals("CTRPRK") ||
           snm.equals("B_MID")  ||
           snm.equals("DP1")    ||
           snm.equals("ASHOOT") ||
           snm.equals("BSHOOT") ||
           snm.equals("DFNPRK"))
        {
            doCorrect = false;
        }
        if(doCorrect) drvTrn.driveToTarget(0.13, dtlCorrect);

        drvTrn.setCurrPt(ept);

        RobotLog.ii("SJH", "Completed move %s. Time: %6.3f HDG: %4d",
                seg.getName(), timer.time(), robot.getGyroFhdg());
    }


    private void doEncoderTurn(double fHdg, int thresh, String prefix)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        drvTrn.setBusyAnd(true);
        drvTrn.setInitValues();
        drvTrn.logData(true, prefix);
        int cHdg = drvTrn.curHdg;
        int tHdg = (int) Math.round(fHdg);
        double angle = tHdg - cHdg;
        RobotLog.ii("SJH", "doEncoderTurn CHDG %4d THDG %4d", cHdg, tHdg);

        while (angle <= -180.0) angle += 360.0;
        while (angle >   180.0) angle -= 360.0;
        if(Math.abs(angle) <= 2.0) return;

        RobotLog.ii("SJH", "Turn %5.2f", angle);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        drvTrn.ctrTurnLinear(angle, DEF_ENCTRN_PWR, thresh);
        cHdg = robot.getGyroFhdg();
        RobotLog.ii("SJH", "Completed turn %5.2f. Time: %6.3f CHDG: %4d",
                angle, timer.time(), cHdg);
    }

    private void doEncoderTurn(double fHdg, String prefix)
    {
        doEncoderTurn(fHdg, Drivetrain.TURN_BUSYTHRESH, prefix);
    }

    private void doGyroTurn(double fHdg, String prefix)
    {
        if(!gyroReady) return;
        if(!opModeIsActive() || isStopRequested()) return;

        drvTrn.setInitValues();
        drvTrn.logData(true, prefix);
        int cHdg = drvTrn.curHdg;
        int tHdg = (int) Math.round(fHdg);

        RobotLog.ii("SJH", "doGyroTurn CHDG %4d THDG %4d", cHdg, tHdg);

        if(Math.abs(tHdg-cHdg) < 1.0)
            return;

        timer.reset();
        drvTrn.ctrTurnToHeading(tHdg, DEF_GYRTRN_PWR);

        cHdg = drvTrn.curHdg;
        RobotLog.ii("SJH", "Completed turnGyro %4d. Time: %6.3f CHDG: %4d",
                tHdg, timer.time(), cHdg);
    }

    private BeaconFinder.BeaconSide findPushSide(BeaconFinder.BeaconSide bSide,
                                                 BeaconFinder.BeaconSide rSide)
    {
        int blue_led = 0;
        int red_led  = 1;

        BeaconFinder.BeaconSide pushSide = BeaconFinder.BeaconSide.UNKNOWN;

        if (bSide == rSide) return pushSide;
        if (bSide == BeaconFinder.BeaconSide.UNKNOWN ||
            rSide == BeaconFinder.BeaconSide.UNKNOWN) return pushSide;

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

        RobotLog.ii("SJH", "BEACON BSIDE %s RSIDE %s PSIDE %s", bSide, rSide, pushSide);
        return pushSide;
    }

    private void setPusher(BeaconFinder.BeaconSide pushSide)
    {
        dashboard.displayPrintf(5, "BUTTON: %s", pushSide);
        if (pushSide == BeaconFinder.BeaconSide.LEFT)
        {
            robot.lpusher.setPosition(L_UP_PUSH_POS);
            robot.rpusher.setPosition(R_DN_PUSH_POS);
            RobotLog.ii("SJH", "Pushing left button");
        }
        else if (pushSide == BeaconFinder.BeaconSide.RIGHT)
        {
            robot.rpusher.setPosition(R_UP_PUSH_POS);
            robot.lpusher.setPosition(L_DN_PUSH_POS);
            RobotLog.ii("SJH", "Pushing right button");
        }
        else
        {
            robot.lpusher.setPosition(L_DN_PUSH_POS);
            robot.rpusher.setPosition(R_DN_PUSH_POS);
            RobotLog.ii("SJH", "Not Pushing A Button");
        }
    }

    private boolean do_findAndPushBeacon(boolean push, Segment seg)
    {
        RobotLog.ii("SJH", "FIND BEACON ORDER!!!");
        dashboard.displayPrintf(2, "STATE: %s", "BEACON FIND");
        beaconNum++;
        int timeout = 1000;
        BeaconFinder.BeaconSide blueSide = BeaconFinder.BeaconSide.UNKNOWN;
        BeaconFinder.BeaconSide redSide  = BeaconFinder.BeaconSide.UNKNOWN;
        BeaconFinder.BeaconSide pushSide = BeaconFinder.BeaconSide.UNKNOWN;

        double zPos = 0.0;

        double totXOffset = 0.0;
        double posXOffset = 0.0;
        double hdgXOffset = 0.0;

        double bcnf = 0.0;

        imgProc.startSensing();
        sleep( 50 );

        int desHdg = (int) seg.getPostTurn().doubleValue();

        //doEncoderTurn(desHdg, "push encoderTurn");
        doGyroTurn(desHdg, "push gyroTurn");

        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opModeIsActive()                              &&
               pushSide == BeaconFinder.BeaconSide.UNKNOWN   &&
               itimer.milliseconds() < timeout)
        {
            totXOffset = bd.getBeaconPosX();
            zPos = bd.getBeaconPosZ();
            bcnf = bd.getBeaconConf();
            robot.waitForTick(20);
        }

        drvTrn.setInitValues();

        double cHdg = drvTrn.curHdg;
        while(cHdg < 0) cHdg += 360;
        double hErr = desHdg - cHdg;

        //confirm direction of getBeaconPosX() and posXOffset
        hdgXOffset = zPos * Math.tan( Math.toRadians( hErr ) );
        posXOffset = totXOffset - hdgXOffset;

        //zPos is ~11" when bot is at 52"

        drvTrn.logData(true, "tXoff " + totXOffset);
        drvTrn.logData(true, "hXoff " + hdgXOffset);
        drvTrn.logData(true, "pXoff " + posXOffset);
        drvTrn.logData(true, "zPos  " + zPos);
        drvTrn.logData(true, "Bconf " + bcnf);

        double MAXERR = 5;
        if(Math.abs(posXOffset) > MAXERR && bcnf > 0.3)
        {
            double theta = Math.acos(1- Math.abs(posXOffset/ShelbyBot.BOT_WIDTH));
            double runRad = ShelbyBot.BOT_WIDTH/2;
            double advDist = ShelbyBot.BOT_WIDTH * Math.sin(theta) + 5;
            theta = Math.toDegrees(theta);
            String outstr = String.format(Locale.US, "%3.1f %3.1f %4.1f %4.1f",
                    theta, advDist, posXOffset, zPos);
            drvTrn.logData(true, "ADJUST " + outstr);
            if(posXOffset > 0)
            {
                if(zPos < advDist)
                {
                    theta = -theta;
                }
            }
            else
            {
                if(zPos >= advDist)
                {
                    theta = -theta;
                    runRad = -runRad;
                }
                else
                {
                    runRad = -runRad;
                }
            }

            drvTrn.logData(true, "CURVE " + theta + " " + runRad);
            drvTrn.turn(theta, 0.3, runRad);
            drvTrn.logData(true, "CURVE " + -theta + " " + -runRad);
            drvTrn.turn(-theta, 0.3, -runRad);

            doGyroTurn(desHdg, "push postAdjGyroTurn");
        }

        itimer.reset();
        pushSide = BeaconFinder.BeaconSide.UNKNOWN;
        int numPsideSamples = 1;

        int numLft = 0;
        int numRgt = 0;
        for (int i = 0; i < numPsideSamples; i++)
        {
            BeaconFinder.BeaconSide bs = BeaconFinder.BeaconSide.UNKNOWN;

            while (opModeIsActive() &&
                   bs == BeaconFinder.BeaconSide.UNKNOWN &&
                   itimer.milliseconds() < timeout)
            {
                blueSide = bd.getBluePosSide();
                redSide = bd.getRedPosSide();
                bs = findPushSide(blueSide, redSide);
                zPos = bd.getBeaconPosZ();
                bcnf = bd.getBeaconConf();
                robot.waitForTick(20);
            }
            if(bs == BeaconFinder.BeaconSide.LEFT)  numLft++;
            if(bs == BeaconFinder.BeaconSide.RIGHT) numRgt++;
            if(numPsideSamples > 1) robot.waitForTick(50);
        }
        pushSide = BeaconFinder.BeaconSide.UNKNOWN;
        if(numLft > numRgt) pushSide = BeaconFinder.BeaconSide.LEFT;
        if(numRgt > numLft) pushSide = BeaconFinder.BeaconSide.RIGHT;
        if(numLft > 0 && numRgt > 0)
        {
            drvTrn.logData(true, "BEACON_MIX " + numLft + " " + numRgt);
        }

        drvTrn.logData(true, "Bside " + blueSide.toString());
        drvTrn.logData(true, "Rside " + redSide.toString());
        drvTrn.logData(true, "Pside " + pushSide.toString());
        drvTrn.logData(true, "zPos  " + zPos);
        drvTrn.logData(true, "Bconf " + bcnf);

//        if(pushSide != BeaconFinder.BeaconSide.UNKNOWN)
//        {
//            imgProc.snapImage(beaconNum);
//        }

        imgProc.stopSensing();

        doGyroTurn(desHdg, "push gyroTurn2");

        setPusher(pushSide);

        if(push && pushSide != BeaconFinder.BeaconSide.UNKNOWN)
        {
            double pushAdd = 4.0;
            if(alliance == Field.Alliance.BLUE) pushAdd = 5.5;
            double tgtDist = zPos + pushAdd;
            Point2d touchStart = seg.getTgtPt();
            double touchX = touchStart.getX();
            double touchY = touchStart.getY();

            if (alliance == Field.Alliance.RED)
            {
                touchX -= tgtDist;
            }
            else
            {
                touchY += tgtDist;
            }

            Point2d touchEnd = new Point2d("TCHEND", touchX, touchY);

            String pushStr = String.format(Locale.US, "PUSHING %s dist %4.1f",
                    pushSide.toString(), tgtDist);
            drvTrn.logData(true, pushStr);
            int dcount;
            robot.setDriveDir(ShelbyBot.DriveDir.PUSHER);

            dcount = drvTrn.driveToPointLinear(touchEnd, 0.2,
                    Drivetrain.Direction.FORWARD, desHdg);

            double actDist = drvTrn.countsToDistance(dcount);

            String actDistStr = String.format(Locale.US, "Actual dist %4.1f",
                    actDist);
            drvTrn.logData(true, pushStr);

            double actTouchX = touchStart.getX();
            double actTouchY = touchStart.getY();

            if (alliance == Field.Alliance.RED)
            {
                //actTouchX -= actDist;
                //actTouchX = drvTrn.estPos.getX();
                actTouchX = -59.0;
            }
            else
            {
                //actTouchY += actDist;
                //actTouchY = drvTrn.estPos.getY();
                actTouchY = 59.0;
                //touchStart.setY(51.0);
            }

            Point2d actTouchPt = new Point2d(actTouchX, actTouchY);
            drvTrn.logData(true, "AT PUSH " + actTouchPt.toString());
            drvTrn.setCurrPt(actTouchPt, true);
            drvTrn.driveToPointLinear(touchStart, 0.2,
                    Drivetrain.Direction.REVERSE, desHdg);
            drvTrn.logData(true, "PUSHED?");
        }

        setPusher(BeaconFinder.BeaconSide.UNKNOWN);

        return pushSide != BeaconFinder.BeaconSide.UNKNOWN;
    }

    private void do_findAndPushBeacon(Point2d endPt)
    {
        RobotLog.ii("SJH", "/BEACON/ > THE HUNT FOR BEACON");

        String beaconStep = "FIND";
        String driveStep = "INIT";

        BeaconFinder.BeaconSide blueSide = BeaconFinder.BeaconSide.UNKNOWN;
        BeaconFinder.BeaconSide redSide  = BeaconFinder.BeaconSide.UNKNOWN;
        BeaconFinder.BeaconSide pushSide = BeaconFinder.BeaconSide.UNKNOWN;

        boolean allDone = false, skipDrive = false;
        double baseSpeed = 0.1;
        double bConf = 0;
        double zPos = 0, xPos = 0, nPos = 0, rDv = 0, lDv = 0;
        double tPow = 0, nAng = 0;
        double cHdg, hErr, nOff, mDir = 1, dDist = 0.0;
        double curDistCount = 0.0;
        double curDist = 0.0;

        imgProc.startSensing();

        robot.setDriveDir(ShelbyBot.DriveDir.PUSHER);

        sleep( 50 );

        double desHdg = 180;
        if (alliance == Field.Alliance.BLUE) desHdg = 90;

        doGyroTurn(desHdg, "push encoderTurn" );

        drvTrn.stopAndReset();
        drvTrn.setInitValues();

        int startLpos = drvTrn.initLpos;
        int startRpos = drvTrn.initRpos;

        while( opModeIsActive() && !allDone ) {

            drvTrn.setCurValues();
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
                        RobotLog.ii("SJH", "/BEACON/FIND > CONFIDENCE HIGH, MOVE ALONG" );
                    }
                    else
                    {
                        beaconStep = "LEAVE";
                        RobotLog.ii("SJH", "/BEACON/FIND > CONFIDENCE LOW, NEXT STEP");
                    }

                    break;

                case "FORWARD":

                    imgProc.logDebug();
                    imgProc.logTelemetry();

                    blueSide = bd.getBluePosSide();
                    redSide  = bd.getRedPosSide();

                    bConf = bd.getBeaconConf();

                    switch ( driveStep ) {
                        case "TRY2":
                            RobotLog.ii("SJH", "/BEACON/TRY2" );
                            mDir = -1;

                        case "INIT":

                            zPos = bd.getBeaconPosZ();
                            xPos = bd.getBeaconPosX();
                            drvTrn.logData(true, "INIT X:" + xPos + " Z:" + zPos);

                            if ( zPos < 12.0 && mDir == 1 && Math.abs( xPos ) > 1.5 ) {
                                mDir = -1;
                            }

                            nOff = zPos * Math.tan( Math.toRadians( hErr ) );
                            nPos = xPos + nOff;
                            nAng = Math.atan( nPos / 9.0 );
                            dDist = zPos / ( Math.cos( nAng )  );
                            tPow = nAng / Math.PI;

                            dl.addField("INIT");
                            dl.addField(xPos);
                            dl.addField(zPos);
                            dl.addField(nOff);
                            dl.addField(nPos);
                            dl.addField(nAng);
                            dl.addField(tPow);
                            dl.newLine();

                            rDv = mDir * Range.clip( baseSpeed + tPow, -0.45, 0.45 );
                            lDv = mDir * Range.clip( baseSpeed - tPow, -0.45, 0.45 );

                            RobotLog.ii("SJH", "/BEACON/INIT > nOff: %5.2f, nPos: %5.2f, nAng: %5.2f, dDist: %5.2f",
                                    nOff, nPos, nAng, dDist );

                            if ( Math.abs( xPos ) > 1.25 ) {
                                drvTrn.stopMotion();
                                drvTrn.moveInit(lDv, rDv);
                            } else {
                                skipDrive = true;
                            }

                            driveStep = "CENTER";
                            break;

                        case "CENTER":

                            RobotLog.ii("SJH", "/BEACON/CENTER > r: %5.2f, l: %5.2f, d: %5.2f, a: %5.2f"
                                    , rDv, lDv, curDist, hErr );

                            if (Math.abs(curDist) > dDist || skipDrive)
                            {
                                drvTrn.stopMotion();

                                doEncoderTurn(desHdg, "push ctrEncoderTurn");
                                doGyroTurn(desHdg, "push ctrGyroTurn");

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
                        RobotLog.ii("SJH", "/BEACON/FORWARD > FOUND %s ON %s", alliance, pushSide );
                    }

                    setPusher(pushSide);

                    if ( driveStep.equals("READY") )
                    {
                        drvTrn.logData(true, "READY pushSide: " + pushSide.toString());
                        if ( pushSide == BeaconFinder.BeaconSide.UNKNOWN )
                        {
                            beaconStep = "BACKUP";
                            RobotLog.ii("SJH", "/BEACON/FORWARD > NO SIDE DETECTED, EXITING AT %5.2f ",
                                    curDistCount );
                        }
                        else
                        {
                            beaconStep = "PUSH";
                            RobotLog.ii("SJH", "/BEACON/FORWARD > TIME TO PUSH BUTTON ON THE %s",
                                    pushSide );
                        }
                    }

                    break;

                case "PUSH":

                    RobotLog.ii("SJH", "/BEACON/PUSH > GOING TO PUSH BUTTON" );

                    drvTrn.logData(false, "PUSH " + pushSide.toString());
                    int c = drvTrn.driveDistanceLinear(14.0, baseSpeed,
                                                       Drivetrain.Direction.FORWARD,
                                                       (int)desHdg);

                    RobotLog.ii("SJH", "/BEACON/PUSH > PUSHED THE BUTTON" );
                    beaconStep = "BACKUP";

                    break;

                case "BACKUP":

                    RobotLog.ii("SJH", "/BEACON/BACKUP > BACKING UP %4.1f", curDistCount );

                    int lDcnt = drvTrn.curLpos - startLpos;
                    int rDcnt = drvTrn.curRpos - startRpos;
                    double backDist = drvTrn.countsToDistance((lDcnt + rDcnt) / 2.0);
                    backDist = Math.abs(backDist);

                    drvTrn.logData(false, "BACKUP " + backDist);
                    if(backDist > 0.5)
                    {
                        drvTrn.driveDistanceLinear(backDist, baseSpeed,
                                Drivetrain.Direction.REVERSE, (int) desHdg);
                    }
                    drvTrn.setCurrPt(endPt);

                    RobotLog.ii("SJH", "/BEACON/BACKUP > BACKED UP" );
                    robot.lpusher.setPosition(L_DN_PUSH_POS);
                    robot.rpusher.setPosition(R_DN_PUSH_POS);
                    beaconStep = "LEAVE";

                    break;

                case "LEAVE":

                    allDone = true;
                    drvTrn.logData(true, "LEAVE");
                    RobotLog.ii("SJH", "/BEACON/LEAVE > MOVING ALONG" );
                    break;

            }

            robot.waitForTick( 10 );
        }

        imgProc.stopSensing();

        drvTrn.stopMotion();
        drvTrn.setEndValues("DONE FINDPUSH");

        RobotLog.ii("SJH", "/BEACON/ > MISSION COMPLETE");
    }

    private void do_shoot()
    {
        RobotLog.ii("SJH", "SHOOT!!!");
        dashboard.displayPrintf(2, "STATE: %s", "SHOOT");
        ElapsedTime stimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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
        FtcChoiceMenu<Field.StartPos> startPosMenu =
                new FtcChoiceMenu<>("START:", null, this);
        FtcChoiceMenu<Field.BeaconChoice> pushMenu =
                new FtcChoiceMenu<>("PUSH:", startPosMenu, this);
        FtcChoiceMenu<Field.ParkChoice> parkMenu   =
                new FtcChoiceMenu<>("PARK:", pushMenu, this);
        FtcChoiceMenu<Field.Alliance> allianceMenu =
                new FtcChoiceMenu<>("ALLIANCE:", parkMenu, this);
        FtcValueMenu delayMenu     = new FtcValueMenu("DELAY:", allianceMenu, this,
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
        parkMenu.addChoice("DEFEND", Field.ParkChoice.DEFEND_PARK, allianceMenu);

        allianceMenu.addChoice("RED",  Field.Alliance.RED,  delayMenu);
        allianceMenu.addChoice("BLUE", Field.Alliance.BLUE, delayMenu);

        FtcMenu.walkMenuTree(startPosMenu, this);

        startPos = startPosMenu.getCurrentChoiceObject();
        beaconChoice = pushMenu.getCurrentChoiceObject();
        parkChoice = parkMenu.getCurrentChoiceObject();
        alliance = allianceMenu.getCurrentChoiceObject();
        delay = delayMenu.getCurrentValue();

        int lnum = 3;
        dashboard.displayPrintf(lnum++, "START: %s", startPos);
        dashboard.displayPrintf(lnum++, "PUSH: %s", beaconChoice);
        dashboard.displayPrintf(lnum++, "PARK: %s", parkChoice);
        dashboard.displayPrintf(lnum++, "ALLIANCE: %s", alliance);
    }

    private void setupLogger()
    {
        if (logData)
        {
            Date day = new Date();
            Calendar cal = Calendar.getInstance();
            cal.setTime(day);
            String dayStr = String.format(Locale.US, "%d%d%d_%02d%02d_%s",
                    cal.get(Calendar.YEAR),
                    cal.get(Calendar.MONTH),
                    cal.get(Calendar.DAY_OF_MONTH),
                    cal.get(Calendar.HOUR_OF_DAY),
                    cal.get(Calendar.MINUTE),
                    alliance.toString() + "_auton");
            dl = new DataLogger(dayStr);
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
    private final static double R_DN_PUSH_POS = 0.05;
    private final static double L_UP_PUSH_POS = 0.05;
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

    private boolean useFly2Light = false;
    private boolean useImageLoc  = false;

    private boolean firstInState = true;

    private DataLogger dl;
    private boolean logData = true;

    private int postSleep = 150;

    private int beaconNum = 0;
    private int colSegNum = 0;
    private static int numBeacons = 2;
}

