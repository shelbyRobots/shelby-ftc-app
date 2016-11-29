
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.widget.TextView;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@SuppressWarnings({"unused", "ForLoopReplaceableByForEach"})
@Autonomous(name="AutonShelby", group="Auton")
//@Disabled
public class FtcAutoShelby extends FtcOpMode implements FtcMenu.MenuButtons
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
        if(drvTrn != null) drvTrn.stopAndReset();
    }

    private void setup()
    {
        dashboard.displayPrintf(2, "STATE: %s", "INITIALIZING - PLEASE WAIT FOR MENU");
        hardwareMap.logDevices();
        robot.init(hardwareMap);

        robot.colorSensor.enableLed(true);
        robot.colorSensor.enableLed(false);
        sleep(100);
        robot.colorSensor.enableLed(true);
        sleep(100);
        robot.colorSensor.enableLed(false);

        tracker = new ImageTracker();

        if (robot.leftMotor  != null &&
            robot.rightMotor != null &&
            robot.gyro       != null)
        {
            drvTrn.init(robot.leftMotor, robot.rightMotor, robot.gyro);
            drvTrn.setOpMode(getInstance());

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
            DEF_SHT_PWR = 0.85;
        }

        Points pts = new Points(startPos, alliance, beaconChoice, parkChoice);
        pathSegs = pts.getSegments();

        initHdg = pathSegs[0].getFieldHeading();

        DbgLog.msg("SJH ROUTE: \n" + pts.toString());

        Point2d currPoint = pathSegs[0].getStrtPt();
        drvTrn.setCurrPt(currPoint);
        drvTrn.setInitHdg(initHdg);

        timer.reset();
        DbgLog.msg("SJH Start %s. Time: %6.3f", currPoint, timer.time());

        dashboard.displayPrintf(3, "PATH: Start at %s %6.3f", currPoint,
                                                              timer.seconds());

        dashboard.displayPrintf(6, "GHDG: %d",
                robot.gyro.getIntegratedZValue());
    }

    private void do_main_loop()
    {
        DbgLog.msg("SJH: Delaying for %4.2f seconds", delay);
        ElapsedTime delayTimer = new ElapsedTime();
        while (delayTimer.seconds() < delay)
        {
            idle();
        }

        DbgLog.msg("SJH: Done delay");

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

            DbgLog.msg("SJH Planned pos: %s %s",
                    pathSegs[i].getTgtPt(),
                    pathSegs[i].getFieldHeading());

            switch (curSeg.getAction())
            {
                case SHOOT:
                    do_shoot();
                    break;
                case SCAN_IMAGE:
                    if (scanImage && findSensedLoc())
                    {
                        DbgLog.msg("SJH Sensed pos: %s %s",
                                curPos, curHdg);
                    }
                    break;
                case FIND_BEACON:

                    SkipNextSegment = !do_findBeaconOrder(true);

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
        Point2d pt = seg.getTgtPt();
        if(robot.colorSensor != null && seg.getTgtType() == Segment.TargetType.COLOR)
        {
            DbgLog.msg("SJH: Turning on colorSensor LED");
            robot.colorSensor.enableLed(true);
            DcMotor.RunMode lRunMode = robot.leftMotor.getMode();
            DcMotor.RunMode rRunMode = robot.rightMotor.getMode();
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int lpos = robot.leftMotor.getCurrentPosition();
            int rpos = robot.rightMotor.getCurrentPosition();
            int segCounts = drvTrn.distanceToCounts(seg.getLength());

            drvTrn.move(seg.getSpeed());

            while(opModeIsActive() &&
                  !isStopRequested())
            {
                if (robot.colorSensor.red()   > RED_THRESH &&
                    robot.colorSensor.green() > GRN_THRESH &&
                    robot.colorSensor.blue()  > BLU_THRESH)
                {
                    drvTrn.stopAndReset();
                    robot.colorSensor.enableLed(false);
                    DbgLog.msg("SJH: FOUND LINE");
                    break;
                }
                else if(robot.leftMotor.getCurrentPosition()  - lpos > (int)(segCounts * 1.2) ||
                        robot.rightMotor.getCurrentPosition() - rpos > (int)(segCounts * 1.2))
                {
                    drvTrn.stopAndReset();
                    robot.colorSensor.enableLed(false);
                    DbgLog.msg("SJH: Backing up a bit");
                    drvTrn.driveDistanceLinear(2.0, 0.3, Drivetrain.Direction.REVERSE);
                    DbgLog.msg("SJH: REACHED OVERRUN PT");
                    break;
                }
            }
            //sleep(1500);
        }
        else
        {
            int targetHdg = (int)Math.round(seg.getFieldHeading());
            drvTrn.driveToPointLinear(pt, speed, ddir, targetHdg);
        }

        RobotLog.ii("SJH", "Completed move %s. Time: %6.3f HDG: %5.2f",
                seg.getName(), timer.time(), getGryoFhdg());
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
        drvTrn.ctrTurnLinear(angle,DEF_TRN_PWR);
        cHdg = getGryoFhdg();
        DbgLog.msg("SJH Completed turn %5.2f. Time: %6.3f CHDG: %5.2f",
                angle, timer.time(), cHdg);
    }

    private void doEncoderTurn(Segment seg)
    {
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
        drvTrn.ctrTurnLinear(angle,DEF_TRN_PWR);
        cHdg = getGryoFhdg();
        DbgLog.msg("SJH Completed turn %5.2f. Time: %6.3f CHDG: %5.2f",
                angle, timer.time(), cHdg);
    }

    private void doPostTurn(double fHdg)
    {
        double cHdg = getGryoFhdg();
        double tHdg = Math.round(fHdg);

        DbgLog.msg("SJH: do post GyroTurn CHDG %4.1f THDG %4.1f",
                cHdg,
                tHdg);

        if(Math.abs(tHdg-cHdg) <= 1.0)
            return;

        timer.reset();
        drvTrn.ctrTurnToHeading(tHdg, DEF_TRN_PWR);

        cHdg = getGryoFhdg();
        DbgLog.msg("SJH Completed post turnGyro %5.2f. Time: %6.3f CHDG: %5.2f",
                tHdg, timer.time(), cHdg);
    }

    private void doTurn(Segment seg)
    {
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
        drvTrn.ctrTurnToHeading(tHdg, DEF_TRN_PWR);

        cHdg = getGryoFhdg();
        DbgLog.msg("SJH Completed turnGyro %5.2f. Time: %6.3f CHDG: %5.2f",
                tHdg, timer.time(), cHdg);
    }

    private boolean findSensedLoc()
    {
        DbgLog.msg("SJH findSensedLoc");
        dashboard.displayPrintf(2, "STATE: %s", "FIND IMG LOC");
        curPos = null;
        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        tracker.setActive(true);
        Point2d sensedBotPos = null;
        double  sensedFldHdg = pathSegs[0].getFieldHeading();
        while(opModeIsActive() && itimer.milliseconds() < 2000)
        {
            tracker.updateRobotLocationInfo();
            sensedBotPos = tracker.getSensedPosition();

            if(sensedBotPos != null)
            {
                curPos = sensedBotPos;
                sensedFldHdg = tracker.getSensedFldHeading();
                curHdg = sensedFldHdg;
                break;
            }
            sleep(50);
        }

        tracker.setActive(false);

        if ( sensedBotPos != null )
        {
            DbgLog.msg("Image based location: %s %5.2f", sensedBotPos, sensedFldHdg);
            dashboard.displayPrintf(4, "SENSLOC: %s %5.2f",
                    sensedBotPos, sensedFldHdg);
        }
        else
        {
            dashboard.displayPrintf(4, "SENSLOC: %s", "NO VALUE");
        }

        return (curPos != null);
    }

    private void do_beacon_push_integrated()
    {
        RobotLog.ii("SJH", "FIND/PUSH");
        dashboard.displayPrintf(2, "STATE: %s", "BEACON FIND/PUSH");
        do_findBeaconOrder(true);
    }

    private boolean do_findBeaconOrder(boolean push)
    {
        DbgLog.msg("SJH: FIND BEACON ORDER!!!");
        dashboard.displayPrintf(2, "STATE: %s", "BEACON FIND");
        int timeout = 2000;
        BeaconFinder.LightOrder ord = BeaconFinder.LightOrder.UNKNOWN;
        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        tracker.setFrameQueueSize(10);
        tracker.setActive(true);
        while (opModeIsActive() &&
               (ord == BeaconFinder.LightOrder.UNKNOWN ||
                ord == BeaconFinder.LightOrder.RED_RED ||
                ord == BeaconFinder.LightOrder.BLUE_BLUE) &&
               itimer.milliseconds() < timeout)
        {
            Bitmap bmap = tracker.getImage();
            if(bmap != null)
            {
                bd.setBitmap(bmap);
                ord = bd.getLightOrder();
            }
            sleep(50);
        }
        tracker.setActive(false);
        tracker.setFrameQueueSize(0);

        if (ord != BeaconFinder.LightOrder.UNKNOWN)
        {
            DbgLog.msg("SJH: Found Beacon!!! " + ord);
        }

        if (ord == BeaconFinder.LightOrder.BLUE_RED)
        {
            if (alliance == Field.Alliance.BLUE)
            {
                bSide = ButtonSide.LEFT;
            }
            else
            {
                bSide = ButtonSide.RIGHT;
            }
        }
        else if (ord == BeaconFinder.LightOrder.RED_BLUE)
        {
            if (alliance == Field.Alliance.BLUE)
            {
                bSide = ButtonSide.RIGHT;
            }
            else
            {
                bSide = ButtonSide.LEFT;
            }
        }

        DbgLog.msg("SJH: Gonna push button " + bSide);
        dashboard.displayPrintf(5, "BUTTON: %s", bSide);

        if(push)
        {
            do_pushButton(bSide);
        }
        return bSide != ButtonSide.UNKNOWN;
    }

    private void do_pushButton(ButtonSide bside)
    {
        DbgLog.msg("SJH: PUSH BUTTON!!!");
        dashboard.displayPrintf(2, "STATE: %s", "BUTTON PUSH");
        if (bside == ButtonSide.LEFT)
        {
            robot.pusher.setPosition(LFT_PUSH_POS);
            DbgLog.msg("SJH: Pushing left button");
        }
        else if (bside == ButtonSide.RIGHT)
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
    }

    private enum ButtonSide
    {
        UNKNOWN,
        LEFT,
        RIGHT
    }

    private enum Team
    {
        SONIC,
        SNOWMAN
    }

    private final static double ZER_PUSH_POS = 0.0;
    private final static double RGT_PUSH_POS = 0.1;
    private final static double LFT_PUSH_POS = 0.9;
    private final static double CTR_PUSH_POS = 0.5;

     //private final static double DEF_DRV_PWR  = 0.7;
    private final static double DEF_TRN_PWR  = 0.45;

    private final static double DEF_SWP_PWR = 1.0;
    private final static double DEF_ELV_PWR = 0.5;

    private Segment[] pathSegs;

    private ShelbyBot   robot = new ShelbyBot();
    private ElapsedTime timer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private ImageTracker tracker;
    private BeaconDetector bd = new BeaconDetector();
    private ButtonSide bSide = ButtonSide.UNKNOWN;

    private static Point2d curPos;
    private static double  curHdg;

    private static Field.AutoStrategy autoStrategy =
            Field.AutoStrategy.SHOOT_PARKCNTR;

    private static Field.StartPos startPos = Field.StartPos.START_A;
    private static Field.BeaconChoice beaconChoice = Field.BeaconChoice.NEAR;
    private static Field.ParkChoice parkChoice = Field.ParkChoice.CENTER_PARK;
    private static Field.Alliance alliance = Field.Alliance.RED;
    private static Team team = Team.SONIC;
    private static double DEF_SHT_PWR = 0.9;

    private HalDashboard dashboard;

    private double initHdg = 0.0;

    private boolean scanImage = false;
    private boolean useImageLoc = false;
    private boolean usePostTurn = true;

    private boolean gyroReady;

    private int RED_THRESH = 15;
    private int GRN_THRESH = 15;
    private int BLU_THRESH = 15;

    private double delay = 0.0;
}
