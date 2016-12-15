
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.widget.TextView;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@SuppressWarnings({"unused", "ForLoopReplaceableByForEach"})
@Autonomous(name="AutonShelby", group="Auton")
//@Disabled
public class FtcAutoShelby extends FtcOpMode implements FtcMenu.MenuButtons, CameraBridgeViewBase.CvCameraViewListener2
{
    public FtcAutoShelby()
    {
        super();
    }

    @Override
    public void initRobot()
    {
        telemetry.addData("_","PLEASE WAIT - STARTING");
        telemetry.update();
        dashboard = getDashboard();
        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);
        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));
        setup();
    }

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        do_main_loop();
    }

    @Override
    public void runPeriodic(double elapsedTime)
    {
        if(robot.gyro != null)
        {
            dashboard.displayPrintf(6, "GHDG: %d",
                    robot.gyro.getIntegratedZValue());
        }
    }

    @Override
    public void stopMode()
    {
        if (openCVCamera != null)
        {
            openCVCamera.disableView();
        }

        if(drvTrn != null) drvTrn.stopAndReset();
    }

    private void setup()
    {
        dashboard.displayPrintf(2, "STATE: %s", "INITIALIZING - PLEASE WAIT FOR MENU");
        DbgLog.msg("SJH: SETUP");
        hardwareMap.logDevices();
        robot.init(hardwareMap);

        DbgLog.msg("SJH: I2C Controller version %d",
                robot.colorSensor.getI2cController().getVersion());

        DbgLog.msg("SJH: COLOR_SENSOR");
        DbgLog.msg("SJH:  ConnectionInfo %s", robot.colorSensor.getConnectionInfo());
        DbgLog.msg("SJH:  I2cAddr %s", Integer.toHexString(robot.colorSensor.getI2cAddress().get8Bit()));
        DbgLog.msg("SJH:  I2cAddr %s", Integer.toHexString(robot.colorSensor.getI2cAddress().get7Bit()));

        DbgLog.msg("SJH: GYRO_SENSOR");
        DbgLog.msg("SJH:  ConnectionInfo %s", robot.gyro.getConnectionInfo());
        DbgLog.msg("SJH:  I2cAddr %s", Integer.toHexString(robot.gyro.getI2cAddress().get8Bit()));
        DbgLog.msg("SJH:  I2cAddr %s", Integer.toHexString(robot.gyro.getI2cAddress().get7Bit()));

        robot.colorSensor.enableLed(true);
        sleep(50);

        turnColorOff();

        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS: {
                        openCVCamera.enableView();
                    }
                    break;
                    default: {
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }
        };

        openCVCamera = (JavaCameraView) ((Activity) hardwareMap.appContext).findViewById(R.id.surfaceView);
        openCVCamera.setVisibility(CameraBridgeViewBase.VISIBLE);
        openCVCamera.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_FRONT);
        openCVCamera.setCvCameraViewListener(this);

        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, hardwareMap.appContext, mLoaderCallback);
        } else {
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        if (robot.leftMotor  != null &&
            robot.rightMotor != null &&
            robot.gyro       != null)
        {
            drvTrn.init(robot.leftMotor, robot.rightMotor, robot.gyro);
            drvTrn.setOpMode(getInstance());
            robot.setOpMode(getInstance());

            int lms = robot.leftMotor.getMaxSpeed();
            int rms = robot.rightMotor.getMaxSpeed();
            DbgLog.msg("SJH: MaxSpeeds %d %d", lms, rms);

            DbgLog.msg("SJH: Starting gyro calibration");
            robot.gyro.calibrate();

            // make sure the gyro is calibrated before continuing
            ElapsedTime gyroTimer = new ElapsedTime();
            double gyroInitTimout = 5.0;
            boolean gyroCalibTimedout = false;
            gyroTimer.reset();
            while (!isStopRequested() &&
                   robot.gyro.isCalibrating())
            {
                sleep(50);
                if(gyroTimer.seconds() > gyroInitTimout)
                {
                    DbgLog.msg("SJH: GYRO INIT TIMED OUT!!");
                    gyroCalibTimedout = true;
                    break;
                }
            }
            DbgLog.msg("SJH: Gyro callibrated in %4.2f seconds", gyroTimer.seconds());

            gyroReady = !gyroCalibTimedout;
            if(gyroReady) robot.gyro.resetZAxisIntegrator();

            drvTrn.setGryoReady(gyroReady);
        }

        doMenus();
        robot.pusher.setPosition(ZER_PUSH_POS);

        if(team == Team.SNOWMAN)
        {
            DEF_SHT_PWR = 0.75;
        }

        Points pts = new Points(startPos, alliance, beaconChoice, parkChoice);
        pathSegs = pts.getSegments();

        initHdg = pathSegs[0].getFieldHeading();

        DbgLog.msg("SJH ROUTE: \n" + pts.toString());

        Point2d currPoint = pathSegs[0].getStrtPt();
        drvTrn.setCurrPt(currPoint);
        drvTrn.setInitHdg(initHdg);

        DbgLog.msg("SJH Start %s.", currPoint);
        dashboard.displayPrintf(3, "PATH: Start at %s", currPoint);

        DbgLog.msg("SJH IHDG %4.3f", initHdg);

        ElapsedTime ptimer = new ElapsedTime();
        double pTimeout = 0.1;
        while(!isStarted() && !isStopRequested())
        {
            if(ptimer.seconds() > pTimeout)
            {
                DbgLog.msg("SJH INIT CHDG %d", robot.gyro.getIntegratedZValue());
                dashboard.displayPrintf(6, "GHDG: %d",
                        robot.gyro.getIntegratedZValue());
                ptimer.reset();
            }

        }
    }

    private void do_main_loop()
    {
        timer.reset();

        DbgLog.msg("SJH: STARTING AT %4.2f", timer.seconds());

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
            DbgLog.msg("SJH: Starting segment %s at %4.2f", pathSegs[i].getName(),
                    getOpModeElapsedTime());

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

            doEncoderTurn(curSeg); //quick but rough
            if(gyroReady) doTurn(curSeg); //fine tune using gyro
            DbgLog.msg("SJH: Setting drive tuner to %4.2f", curSeg.getDrvTuner());
            drvTrn.setDrvTuner(curSeg.getDrvTuner());
            doMove(curSeg);
            Double pturn = curSeg.getPostTurn();

            if(usePostTurn && pturn != null)
            {
                DbgLog.msg("SJH POST TURN %s", curSeg.getName());
                doEncoderPostTurn(pturn);
                if(gyroReady) doPostTurn(pturn);
            }

            if(!opModeIsActive() || isStopRequested())
            {
                drvTrn.stopAndReset();
                break;
            }

            DbgLog.msg("SJH Planned pos: %s %s",
                    pathSegs[i].getTgtPt(),
                    pathSegs[i].getFieldHeading());

            switch (curSeg.getAction())
            {
                case SHOOT:
                    do_shoot();
                    break;

                case FIND_BEACON:
                    do_findAndPushBeacon(true);
                    //do_findAndPushBeacon();
                    break;

                case RST_PUSHER:
                    robot.pusher.setPosition(ZER_PUSH_POS);
                    break;

                case NOTHING:
                    break;
            }
        }
    }

    private void doMove(Segment seg)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        String  snm = seg.getName();
        Point2d spt = seg.getStrtPt();
        Point2d ept = seg.getTgtPt();
        double  fhd = seg.getFieldHeading();
        Segment.SegDir dir = seg.getDir();
        double speed = seg.getSpeed();
        double fudge = seg.getDrvTuner();
        Segment.TargetType ttype = seg.getTgtType();

        RobotLog.ii("SJH", "Drive %s %s %s %6.2f %3.2f %s tune: %4.2f %s",
                snm, spt, ept, fhd, speed, dir, fudge, ttype);

        dashboard.displayPrintf(2, "STATE: %s %s %s - %s %6.2f %3.2f %s",
                "DRIVE", snm, spt, ept, fhd, speed, dir);

        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;
        if (dir == Segment.SegDir.REVERSE) ddir = Drivetrain.Direction.REVERSE;
        timer.reset();

        if(robot.colorSensor != null && seg.getTgtType() == Segment.TargetType.COLOR)
        {
            DbgLog.msg("SJH: Turning on colorSensor LED");
            turnColorOn();
            DcMotor.RunMode lRunMode = robot.leftMotor.getMode();
            DcMotor.RunMode rRunMode = robot.rightMotor.getMode();
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int lpos = robot.leftMotor.getCurrentPosition();
            int rpos = robot.rightMotor.getCurrentPosition();
            int segCounts = drvTrn.distanceToCounts(seg.getLength());

            DbgLog.msg("SJH: Color Driving to pt %s at speed %4.2f", ept, speed);
            drvTrn.move(speed);

            while(opModeIsActive() &&
                  !isStopRequested())
            {
                int r = robot.colorSensor.red();
                int g = robot.colorSensor.green();
                int b = robot.colorSensor.blue();

                DbgLog.msg("SJH: RGB %d %d %d", r, g, b);

                int totColor = r + g + b;

                if (totColor > COLOR_THRESH)
                {
                    drvTrn.stopAndReset();
                    DbgLog.msg("SJH: FOUND LINE");
                    turnColorOff();
                    drvTrn.setCurrPt(ept);
                    break;
                }
                else if(robot.leftMotor.getCurrentPosition()  - lpos > (int)(segCounts * 1.2) ||
                        robot.rightMotor.getCurrentPosition() - rpos > (int)(segCounts * 1.2))
                {
                    drvTrn.stopAndReset();
                    DbgLog.msg("SJH: REACHED OVERRUN PT - Backing up a bit");
                    turnColorOff();
                    drvTrn.driveDistanceLinear(3.0, 0.3, Drivetrain.Direction.REVERSE);
                    drvTrn.setCurrPt(ept);
                    break;
                }

                robot.waitForTick(5);
            }
        }
        else
        {
            int targetHdg = (int)Math.round(seg.getFieldHeading());
            if(dir == Segment.SegDir.REVERSE)
            {
                targetHdg += 180;
                while(targetHdg >   180) { targetHdg -= 180; }
                while(targetHdg <= -180) { targetHdg += 180; }
            }
            drvTrn.driveToPointLinear(ept, speed, ddir, targetHdg);
        }

        RobotLog.ii("SJH", "Completed move %s. Time: %6.3f HDG: %5.2f",
                seg.getName(), timer.time(), getGryoFhdg());
    }

    private void turnColorOn()
    {
        ModernRoboticsI2cColorSensor cs = robot.colorSensor;
        cs.getI2cController().registerForI2cPortReadyCallback(robot.colorSensor,
                                                 robot.getColorPort());

        sleep(50);
        cs.enableLed(true);
    }

    private void turnColorOff()
    {
        ModernRoboticsI2cColorSensor cs = robot.colorSensor;
        cs.enableLed(false);
        sleep(50);
        cs.getI2cController().deregisterForPortReadyCallback(robot.getColorPort());
    }

    private double getGryoFhdg()
    {
        double cHdg = robot.gyro.getIntegratedZValue() + initHdg;

        while (cHdg <= -180.0) cHdg += 360.0;
        while (cHdg >   180.0) cHdg -= 360.0;

        return cHdg;
    }

    private void doEncoderPostTurn(double fHdg)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        double cHdg = getGryoFhdg();
        double tHdg = Math.round(fHdg);
        double angle = tHdg - cHdg;
        DbgLog.msg("SJH: doEncoderPostTurn CHDG %4.1f THDG %4.1f",
                cHdg,
                tHdg);

        while (angle <= -180.0) angle += 360.0;
        while (angle >   180.0) angle -= 360.0;
        if(Math.abs(angle) <= 5.0) return;

        DbgLog.msg("SJH: Turn %5.2f", angle);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        drvTrn.ctrTurnLinear(angle, DEF_ENCTRN_PWR);
        cHdg = getGryoFhdg();
        DbgLog.msg("SJH Completed turn %5.2f. Time: %6.3f CHDG: %5.2f",
                angle, timer.time(), cHdg);
    }

    private void doEncoderTurn(Segment seg)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        if (seg.getDir() == Segment.SegDir.REVERSE) return;
        double cHdg = getGryoFhdg();
        double tHdg = Math.round(seg.getFieldHeading());
        double angle = tHdg - cHdg;
        DbgLog.msg("SJH: doEncoderTurn %s CHDG %4.1f THDG %4.1f",
                seg.getName(),
                cHdg,
                tHdg);

        while (angle <= -180.0) angle += 360.0;
        while (angle >   180.0) angle -= 360.0;
        if(Math.abs(angle) <= 5.0) return;

        DbgLog.msg("SJH: Turn %5.2f", angle);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        drvTrn.ctrTurnLinear(angle, DEF_ENCTRN_PWR);
        cHdg = getGryoFhdg();
        DbgLog.msg("SJH Completed turn %5.2f. Time: %6.3f CHDG: %5.2f",
                angle, timer.time(), cHdg);
    }

    private void doPostTurn(double fHdg)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        double cHdg = getGryoFhdg();
        double tHdg = Math.round(fHdg);

        DbgLog.msg("SJH: do post GyroTurn CHDG %4.1f THDG %4.1f",
                cHdg,
                tHdg);

        if(Math.abs(tHdg-cHdg) <= 1.0)
            return;

        timer.reset();
        drvTrn.ctrTurnToHeading(tHdg, DEF_GYRTRN_PWR);

        cHdg = getGryoFhdg();
        DbgLog.msg("SJH Completed post turnGyro %5.2f. Time: %6.3f CHDG: %5.2f",
                tHdg, timer.time(), cHdg);
    }

    private void doTurn(Segment seg)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        double cHdg = getGryoFhdg();
        double tHdg = Math.round(seg.getFieldHeading());
        if(seg.getDir() == Segment.SegDir.REVERSE)
            return;

        DbgLog.msg("SJH: doGyroTurn %s CHDG %4.1f THDG %4.1f",
                seg.getName(),
                cHdg,
                tHdg);

        if(Math.abs(tHdg-cHdg) <= 1.0)
            return;

        timer.reset();
        drvTrn.ctrTurnToHeading(tHdg, DEF_GYRTRN_PWR);

        cHdg = getGryoFhdg();
        DbgLog.msg("SJH Completed turnGyro %5.2f. Time: %6.3f CHDG: %5.2f",
                tHdg, timer.time(), cHdg);
    }

    private BeaconDetector.BeaconSide findPushSide(BeaconDetector.BeaconSide bSide,
                                                   BeaconDetector.BeaconSide rSide)
    {
        BeaconDetector.BeaconSide pushSide = BeaconDetector.BeaconSide.UNKNOWN;

        if (bSide == rSide) return pushSide;

        if      ( alliance == Field.Alliance.BLUE ) pushSide = bSide;
        else if ( alliance == Field.Alliance.RED  ) pushSide = rSide;

        DbgLog.msg("SJH: BEACON BSIDE %s RSIDE %s PSIDE %s", bSide, rSide, pushSide);
        return pushSide;
    }

    private void setPusher(BeaconDetector.BeaconSide pushSide)
    {
        dashboard.displayPrintf(5, "BUTTON: %s", pushSide);
        if (pushSide == BeaconDetector.BeaconSide.LEFT)
        {
            robot.pusher.setPosition(LFT_PUSH_POS);
            DbgLog.msg("SJH: Pushing left button");
        }
        else if (pushSide == BeaconDetector.BeaconSide.RIGHT)
        {
            robot.pusher.setPosition(RGT_PUSH_POS);
            DbgLog.msg("SJH: Pushing right button");
        }
        else
        {
            robot.pusher.setPosition(CTR_PUSH_POS);
            DbgLog.msg("SJH: Not Pushing A Button");
        }
    }

    private boolean do_findAndPushBeacon(boolean push)
    {
        DbgLog.msg("SJH: FIND BEACON ORDER!!!");
        dashboard.displayPrintf(2, "STATE: %s", "BEACON FIND");
        int timeout = 2000;
        BeaconDetector.BeaconSide blueSide = BeaconDetector.BeaconSide.UNKNOWN;
        BeaconDetector.BeaconSide redSide = BeaconDetector.BeaconSide.UNKNOWN;
        BeaconDetector.BeaconSide pushSide = BeaconDetector.BeaconSide.UNKNOWN;

        bd.startSensing();
        //sleep( 200 );

        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opModeIsActive()                              &&
               pushSide == BeaconDetector.BeaconSide.UNKNOWN &&
               itimer.milliseconds() < timeout)
        {
            blueSide = bd.getBluePosSide();
            redSide  = bd.getRedPosSide();
            pushSide = findPushSide(blueSide, redSide);
            robot.waitForTick(34);
        }

        setPusher(pushSide);
        if(push && pushSide != BeaconDetector.BeaconSide.UNKNOWN)
        {
            double tgtDist = 9.5;
            int dcount = drvTrn.driveDistanceLinear(tgtDist, 0.2, Drivetrain.Direction.FORWARD);
            double actDist = drvTrn.countsToDistance(dcount);
            drvTrn.driveDistanceLinear(actDist, 0.5, Drivetrain.Direction.REVERSE);
        }

        bd.stopSensing();

        return pushSide != BeaconDetector.BeaconSide.UNKNOWN;
    }

    private void do_findAndPushBeacon()
    {
        DbgLog.msg("SJH: /BEACON/ > THE HUNT FOR BEACON");

        String beaconStep = "FIND";
        String driveStep = "INIT";

        BeaconDetector.BeaconSide blueSide = BeaconDetector.BeaconSide.UNKNOWN;
        BeaconDetector.BeaconSide redSide = BeaconDetector.BeaconSide.UNKNOWN;
        BeaconDetector.BeaconSide pushSide = BeaconDetector.BeaconSide.UNKNOWN;
        boolean allDone = false;
        double baseSpeed = 0.4;
        double bConf, zPos, xPos, nPos = 0, rDv = 0, lDv = 0;
        double curDistCount = 0.0;
        double bailPos = -1;

        double cRos = 0.03125;
        double dDist = 0;

        double cHdg;
        double hErr;


        bd.startSensing();
        sleep( 500 );

        while(opModeIsActive() && !allDone ) {

            curDistCount = ( robot.leftMotor.getCurrentPosition() +
                                     robot.rightMotor.getCurrentPosition() ) / 2.0;

            switch ( beaconStep )
            {
                case "FIND":

                    bd.logDebug();

                    if ( bd.getBeaconConf() > 0.25 )
                    {
                        beaconStep = "FORWARD";
                        DbgLog.msg("SJH: /BEACON/FIND > CONFIDENCE HIGH, MOVE ALONG" );
                    }
                    else
                    {
                        beaconStep = "LEAVE";
                        DbgLog.msg("SJH: /BEACON/FIND > CONFIDENCE LOW, NEXT STEP" );
                    }

                    break;

                case "FORWARD":

                    bd.logDebug();

                    blueSide = bd.getBluePosSide();
                    redSide = bd.getRedPosSide();

                    bConf = bd.getBeaconConf();
                    zPos = bd.getBeaconPosZ();

                    cHdg = getGryoFhdg() % 90;
                    hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                    curDistCount = ( robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition() ) / 2.0;

                    switch ( driveStep ) {
                        case "INIT":

                            xPos = bd.getBeaconPosX();

                            nPos = xPos / 17.7;
                            dDist = 8.0 / Math.cos( Math.atan( nPos / 16.0 ) ) - 1.0;

                            rDv = Range.clip( baseSpeed + nPos * cRos * 1.8, 0.15, 0.65 );
                            lDv = Range.clip( baseSpeed - nPos * cRos * 1.8, 0.15, 0.65 );

                            DbgLog.msg("SJH: /BEACON/INIT > r: %5.2f, l: %5.2f, xpos: %5.2f", rDv, lDv, xPos );

                            robot.rightMotor.setPower(rDv);
                            robot.leftMotor.setPower(lDv);

                            driveStep = "CENTER";
                            break;

                        case "CENTER":

                            DbgLog.msg("SJH: /BEACON/CENTER > r: %5.2f, l: %5.2f, d: %5.2f, a: %5.2f", rDv, lDv, drvTrn.countsToDistance(curDistCount), hErr );

                            if (drvTrn.countsToDistance(curDistCount) > dDist) {

                                rDv = Range.clip( baseSpeed - nPos * cRos * 1.8, 0.15, 0.65 );
                                lDv = Range.clip( baseSpeed + nPos * cRos * 1.8, 0.15, 0.65 );

                                robot.rightMotor.setPower(rDv);
                                robot.leftMotor.setPower(lDv);

                                driveStep = "ALIGN";
                            }

                            break;

                        case "ALIGN":

                            DbgLog.msg("SJH: /BEACON/ALIGN > r: %5.2f, l: %5.2f, d: %5.2f, a: %5.2f", rDv, lDv, drvTrn.countsToDistance(curDistCount), hErr );

                            if ( drvTrn.countsToDistance(curDistCount) > dDist * 2 )
                            {
                                driveStep = "DRIVE";
                                robot.rightMotor.setPower(0.25);
                                robot.leftMotor.setPower(0.25);

                            }

                            break;

                        case "DRIVE":

                            cHdg = getGryoFhdg() % 90;
                            hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                            curDistCount = (robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition()) / 2.0;

                            rDv = Range.clip( 0.25 - hErr * cRos, 0.15, 0.4);
                            lDv = Range.clip( 0.25 + hErr * cRos, 0.15, 0.4);

                            DbgLog.msg("SJH: /BEACON/DRIVE > r: %5.2f, l: %5.2f, d: %5.2f, a: %5.2f", rDv, lDv, drvTrn.countsToDistance(curDistCount), hErr );

                            robot.rightMotor.setPower(rDv);
                            robot.leftMotor.setPower(lDv);

                            if (bd.getBeaconPosZ() > 0.95) {
                                driveStep = "READY";
                                robot.rightMotor.setPower(0);
                                robot.leftMotor.setPower(0);
                            }

                            break;

                    }

                    // Good when the beacon is in view enough or at least
                    // some driving done.
                    if ( pushSide == BeaconDetector.BeaconSide.UNKNOWN &&
                            blueSide != BeaconDetector.BeaconSide.UNKNOWN &&
                            redSide != BeaconDetector.BeaconSide.UNKNOWN &&
                            driveStep == "ALIGN" )
                    {
                        if ( alliance == Field.Alliance.BLUE )
                        {
                            pushSide = blueSide;
                        }
                        else if ( alliance == Field.Alliance.RED )
                        {
                            pushSide = redSide;
                        }

                        switch ( pushSide ){

                            case LEFT:
                                robot.pusher.setPosition(LFT_PUSH_POS);
                                break;

                            case RIGHT:
                                robot.pusher.setPosition(RGT_PUSH_POS);
                                break;
                        }

                        DbgLog.msg("SJH: /BEACON/FORWARD > FOUND %s ON %s", alliance, pushSide );

                    }

                    if ( zPos > 0.9 || driveStep == "READY" )
                    {
                        if ( pushSide == BeaconDetector.BeaconSide.UNKNOWN )
                        {
                            beaconStep = "BACKUP";
                            bailPos = curDistCount;
                            DbgLog.msg("SJH: /BEACON/FORWARD > NO SIDE DETECTED, EXITING AT %5.2f ", bailPos );
                        }
                        else
                        {
                            beaconStep = "PUSH";
                            DbgLog.msg("SJH: /BEACON/FORWARD > TIME TO PUSH BUTTON ON THE %s", pushSide );
                        }
                    }
                    else if ( bConf < 0.2 && driveStep == "READY" )
                    {
                        beaconStep = "BACKUP";
                        bailPos = curDistCount;
                        DbgLog.msg("SJH: /BEACON/FORWARD > CONFIDENCE DROPPED UNDER THREASHOLD %5.2f", bConf );
                    }
                    else if ( drvTrn.countsToDistance( curDistCount ) > 5.0 && drvTrn.areDriveMotorsStuck() && drvTrn.isBusy() )
                    {
                        beaconStep = "BACKUP";
                        bailPos = curDistCount;
                        DbgLog.msg("SJH: /BEACON/FORWARD > NOT MOVING? YIKES! %5.2f", curDistCount );
                    }

                    break;

                case "PUSH":

                    DbgLog.msg("SJH: /BEACON/PUSH > GOING TO PUSH BUTTON" );
                    robot.rightMotor.setPower( 0.25 );
                    robot.leftMotor.setPower( 0.25 );
                    while ( !drvTrn.areDriveMotorsStuck() )
                        idle();

                    DbgLog.msg("SJH: /BEACON/PUSH > PUSHED THE BUTTON" );
                    beaconStep = "BACKUP";

                    break;

                case "BACKUP":

                    DbgLog.msg("SJH: /BEACON/BACKUP > BACKING UP %4.1f", curDistCount );
                    drvTrn.driveDistanceLinear( drvTrn.countsToDistance( curDistCount ),
                            0.5, Drivetrain.Direction.REVERSE );

                    DbgLog.msg("SJH: /BEACON/BACKUP > BACKED UP" );
                    robot.pusher.setPosition(ZER_PUSH_POS);
                    beaconStep = "LEAVE";

                    break;

                case "LEAVE":

                    allDone = true;
                    DbgLog.msg("SJH: /BEACON/LEAVE > MOVING ALONG" );
                    break;

            }

            robot.waitForTick( 10 );
        }

        bd.stopSensing();
        drvTrn.stopAndReset();

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
        FtcChoiceMenu startPosMenu = new FtcChoiceMenu("START:", null, this);
        FtcChoiceMenu pushMenu     = new FtcChoiceMenu("PUSH:", startPosMenu, this);
        FtcChoiceMenu parkMenu     = new FtcChoiceMenu("PARK:", pushMenu, this);
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("ALLIANCE:", parkMenu, this);
        FtcChoiceMenu teamMenu     = new FtcChoiceMenu("TEAM:", allianceMenu, this);
        //FtcValueMenu powerMenu     = new FtcValueMenu("SHOOTPOWER:", teamMenu, this,
        //                                                    0.0, 1.0, 0.05, 0.55, "%5.2f");
        FtcValueMenu delayMenu     = new FtcValueMenu("DELAY:", teamMenu, this,
                                                             0.0, 20.0, 1.0, 0.0, "%5.2f");

//        strategyMenu.addChoice("Shoot_Push_ParkCenter",      Field.AutoStrategy.SHOOT_PUSH_PARKCNTR,    allianceMenu);
//        strategyMenu.addChoice("Shoot_Push_ParkCorner",      Field.AutoStrategy.SHOOT_PUSH_PARKCRNR,    allianceMenu);
//        strategyMenu.addChoice("Shoot_ParkCenter",           Field.AutoStrategy.SHOOT_PARKCNTR,         allianceMenu);
//        strategyMenu.addChoice("Shoot_ParkCorner",           Field.AutoStrategy.SHOOT_PARKCRNR,         allianceMenu);
//        strategyMenu.addChoice("AngleShoot_Push_ParkCenter", Field.AutoStrategy.ANGSHOOT_PUSH_PARKCNTR, allianceMenu);
//        strategyMenu.addChoice("AngleShoot_Push_ParkCorner", Field.AutoStrategy.ANGSHOOT_PUSH_PARKCRNR, allianceMenu);
//        strategyMenu.addChoice("AngleShoot_ParkCenter",      Field.AutoStrategy.ANGSHOOT_PARKCNTR,      allianceMenu);
//        strategyMenu.addChoice("AngleShoot_ParkCorner",      Field.AutoStrategy.ANGSHOOT_PARKCRNR,      allianceMenu);

        startPosMenu.addChoice("Start_A", Field.StartPos.START_A, pushMenu);
        startPosMenu.addChoice("Start_B", Field.StartPos.START_B, pushMenu);
        pushMenu.addChoice("BOTH", Field.BeaconChoice.BOTH, parkMenu);
        pushMenu.addChoice("NEAR", Field.BeaconChoice.NEAR, parkMenu);
        pushMenu.addChoice("FAR", Field.BeaconChoice.FAR, parkMenu);
        pushMenu.addChoice("NONE", Field.BeaconChoice.NONE, parkMenu);

        parkMenu.addChoice("CENTER", Field.ParkChoice.CENTER_PARK, allianceMenu);
        parkMenu.addChoice("CORNER", Field.ParkChoice.CORNER_PARK, allianceMenu);

        allianceMenu.addChoice("RED",  Field.Alliance.RED, teamMenu);
        allianceMenu.addChoice("BLUE", Field.Alliance.BLUE, teamMenu);

        teamMenu.addChoice("Sonic", Team.SONIC, delayMenu);
        teamMenu.addChoice("Snowman", Team.SNOWMAN, delayMenu);

        //powerMenu.setChildMenu(delayMenu);

        //
        // Walk the menu tree starting with the strategy menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(startPosMenu);
        //
        // Set choices variables.
        //
        //autoStrategy = (Field.AutoStrategy)strategyMenu.getCurrentChoiceObject();

        startPos = (Field.StartPos)startPosMenu.getCurrentChoiceObject();
        beaconChoice = (Field.BeaconChoice)pushMenu.getCurrentChoiceObject();
        parkChoice = (Field.ParkChoice)parkMenu.getCurrentChoiceObject();
        alliance = (Field.Alliance)allianceMenu.getCurrentChoiceObject();
        team = (Team)teamMenu.getCurrentChoiceObject();
        //DEF_SHT_PWR = powerMenu.getCurrentValue();
        delay = delayMenu.getCurrentValue();

        //dashboard.displayPrintf(0, "STRATEGY: %s", autoStrategy);
        dashboard.displayPrintf(0, "START: %s", startPos);
        dashboard.displayPrintf(1, "PUSH: %s", beaconChoice);
        dashboard.displayPrintf(2, "PARK: %s", parkChoice);
        dashboard.displayPrintf(3, "ALLIANCE: %s", alliance);
        dashboard.displayPrintf(4, "TEAM: %s", team);

        DbgLog.msg("SJH: STARTPOS %s", startPos);
        DbgLog.msg("SJH: PUSH     %s", beaconChoice);
        DbgLog.msg("SJH: PARK     %s", parkChoice);
        DbgLog.msg("SJH: ALLIANCE %s", alliance);
        DbgLog.msg("SJH: TEAM     %s", team);
        DbgLog.msg("SJH: DELAY    %4.2f", delay);
    }
    public void onCameraViewStarted(int width, int height) {
        DbgLog.msg("SJH: CAMERA VIEW STARTED");
    }

    public void onCameraViewStopped() {
        DbgLog.msg("SJH: CAMERA VIEW STOPPED");
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat rgb = inputFrame.rgba();
        Mat flip = rgb.clone();

        Core.flip(rgb, flip, 1);
        bd.setImage( flip );

        return bd.drawBeacon();
    }

    private enum Team
    {
        SONIC,
        SNOWMAN
    }

    private final static double ZER_PUSH_POS = 0.1;
    private final static double RGT_PUSH_POS = 0.15;
    private final static double LFT_PUSH_POS = 0.85;
    private final static double CTR_PUSH_POS = 0.1;

    //private final static double DEF_DRV_PWR  = 0.7;
    private final static double DEF_ENCTRN_PWR  = 0.7; //0.70
    private final static double DEF_GYRTRN_PWR = 0.48; //0.48

    private final static double DEF_SWP_PWR = 1.0;
    private final static double DEF_ELV_PWR = 0.5;

    private Segment[] pathSegs;

    private ShelbyBot   robot = new ShelbyBot();
    private ElapsedTime timer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private JavaCameraView openCVCamera;
    private BeaconDetector bd = new BeaconDetector();

    private static Point2d curPos;
    private static double  curHdg;

    private static Field.AutoStrategy autoStrategy =
            Field.AutoStrategy.SHOOT_PARKCNTR;

    private static Field.StartPos startPos = Field.StartPos.START_A;
    private static Field.BeaconChoice beaconChoice = Field.BeaconChoice.NEAR;
    private static Field.ParkChoice parkChoice = Field.ParkChoice.CENTER_PARK;
    private static Field.Alliance alliance = Field.Alliance.RED;
    private static Team team = Team.SONIC;
    private static double DEF_SHT_PWR = 0.85;

    private HalDashboard dashboard;

    private double initHdg = 0.0;

    private boolean scanImage = false;
    private boolean useImageLoc = false;
    private boolean usePostTurn = true;

    private boolean gyroReady;

    private int RED_THRESH = 15;
    private int GRN_THRESH = 15;
    private int BLU_THRESH = 15;

    private int COLOR_THRESH = 25;

    private double delay = 0.0;
}
