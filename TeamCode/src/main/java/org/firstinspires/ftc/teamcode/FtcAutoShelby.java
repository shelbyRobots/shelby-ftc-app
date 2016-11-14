
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.widget.TextView;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import hallib.HalDashboard;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@SuppressWarnings("unused")
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
        robot.gyro.resetZAxisIntegrator();
        do_main_loop();
    }

    @Override
    public void runPeriodic(double elapsedTime)
    {
        dashboard.displayPrintf(6, "GHDG: %d",
                robot.gyro.getIntegratedZValue());
    }

    @Override
    public void stopMode()
    {
        drvTrn.stopAndReset();
    }

    private void setup()
    {
        dashboard.displayPrintf(2, "STATE: %s", "INITIALIZING - PLEASE WAIT FOR MENU");
        hardwareMap.logDevices();
        robot.init(hardwareMap);

        tracker = new ImageTracker();

        doMenus();

        if(team == Team.SNOWMAN)
        {
            DEF_SHT_PWR = SHT_PWR_SNOWMAN;
        }
        else
        {
            DEF_SHT_PWR = SHT_PWR_SONIC;
        }

        if (robot.leftMotor  != null &&
            robot.rightMotor != null &&
            robot.gyro       != null)
        {
            drvTrn.init(robot.leftMotor, robot.rightMotor, robot.gyro);
            drvTrn.setOpMode(getInstance());
            robot.gyro.calibrate();

            // make sure the gyro is calibrated before continuing
            while (!isStopRequested() && robot.gyro.isCalibrating())
            {
                sleep(50);
                idle();
            }
        }

        Points pts = new Points(autoStrategy);
        pathSegs = pts.getSegments(alliance);

        initHdg = pathSegs[0].getFieldHeading();

        DbgLog.msg("SJH ROUTE: \n" + pts.toString());

        Point2d currPoint = pathSegs[0].getStrtPt();
        drvTrn.setCurrPt(currPoint);
        drvTrn.setInitHdg(initHdg);

        robot.pusher.setPosition(RGT_PUSH_POS);

        timer.reset();
        DbgLog.msg("SJH Start %s. Time: %6.3f", currPoint, timer.time());

        dashboard.displayPrintf(3, "PATH: Start at %s %6.3f", currPoint,
                                                              timer.seconds());

        dashboard.displayPrintf(6, "GHDG: %d",
                robot.gyro.getIntegratedZValue());
    }

    private void do_main_loop()
    {
        boolean drive_to_beacon = false;

        for (int i = 0; i < pathSegs.length; ++i)
        {
            if(!opModeIsActive() || isStopRequested())
            {
                drvTrn.stopAndReset();
                break;
            }

            Segment curSeg;
            if(curPos == null)
            {
                curSeg = pathSegs[i];
            }
            else
            {
                curSeg = new Segment("CURSEG", curPos, pathSegs[i].getTgtPt());
            }
            curPos = null;

            doEncoderTurn(curSeg); //quick but rough
            doTurn(curSeg); //fine tune using gyro
            if(curSeg.getAction() != Segment.Action.PUSH ||
               drive_to_beacon)
            {
                doMove(curSeg);
            }

            DbgLog.msg("SJH Planned pos: %s %s",
                    pathSegs[i].getTgtPt(),
                    pathSegs[i].getFieldHeading());

            switch(pathSegs[i].getName())
            {
                case "START_PT":
                    break;
            }
            switch (pathSegs[i].getAction())
            {
                case SHOOT:
                    do_shoot();
                    break;
                case SCAN_IMAGE:
                    if (findSensedLoc())
                    {
                        DbgLog.msg("SJH Sensed pos: %s %s",
                                curPos, curHdg);
                    }
                    if (curPos != null) drvTrn.setCurrPt(curPos);
                    break;
                case FIND_BEACON:
                    drive_to_beacon = do_findBeaconOrder(true);
                    break;
                case RST_PUSHER:
                    drive_to_beacon = false;
                    robot.pusher.setPosition(RGT_PUSH_POS);
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

        RobotLog.ii("SJH", "Drive %s %s %s %6.2f %3.2f %s",
                snm, spt, ept, fhd, speed, dir);

        dashboard.displayPrintf(2, "STATE: %s %s %s - %s %6.2f %3.2f %s",
                "DRIVE", snm, spt, ept, fhd, speed, dir);

        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;
        if (dir == Segment.SegDir.REVERSE) ddir = Drivetrain.Direction.REVERSE;
        timer.reset();
        Point2d pt = seg.getTgtPt();
        drvTrn.driveToPointLinear(pt, speed, ddir);
        RobotLog.ii("SJH", "Completed move %s. Time: %6.3f", seg.getName(), timer.time());
    }

    private double getGryoFhdg()
    {
        double cHdg = robot.gyro.getIntegratedZValue() + initHdg;

        while (cHdg <= -180.0) cHdg += 360.0;
        while (cHdg >   180.0) cHdg -= 360.0;

        return cHdg;
    }

    private void doEncoderTurn(Segment seg)
    {
        if (seg.getDir() == Segment.SegDir.REVERSE) return;
        double cHdg = getGryoFhdg();
        double tHdg = seg.getFieldHeading();
        double angle = tHdg - cHdg;
        DbgLog.msg("SJH: doEncoderTurn %s CHDG %4.1f THDG %4.1f",
                seg.getName(),
                cHdg,
                tHdg);

        while (angle <= -180.0) angle += 360.0;
        while (angle >   180.0) angle -= 360.0;
        if(Math.abs(angle) <= 1.0) return;

        DbgLog.msg("SJH: Turn %5.2f", angle);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        drvTrn.ctrTurnLinear(angle,DEF_TRN_PWR);
        //drvTrn.ctrTurnLinearGyro(angle,DEF_TRN_PWR);
        cHdg = getGryoFhdg();
        DbgLog.msg("SJH Completed turn %5.2f. Time: %6.3f CHDG: %5.2f",
                angle, timer.time(), cHdg);
    }

    private void doTurn(Segment seg)
    {
        double tHdg = seg.getFieldHeading();
        if(seg.getDir() == Segment.SegDir.REVERSE)
            return;
        double cHdg = getGryoFhdg();

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
        while(sensedBotPos == null && itimer.milliseconds() < 1000)
        {
            tracker.updateRobotLocationInfo();
            sensedBotPos = tracker.getSensedPosition();

            if(sensedBotPos != null)
            {
                curPos = sensedBotPos;
                sensedFldHdg = tracker.getSensedFldHeading();
                curHdg = sensedFldHdg;
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
        boolean ready_to_push = false;
        dashboard.displayPrintf(2, "STATE: %s", "BEACON FIND");
        int timeout = 1000;
        BeaconFinder.LightOrder ord = BeaconFinder.LightOrder.UNKNOWN;
        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        tracker.setFrameQueueSize(10);
        tracker.setActive(true);
        while  ((ord == BeaconFinder.LightOrder.UNKNOWN ||
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
            ready_to_push = true;
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
            ready_to_push = true;
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

        return ready_to_push;
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
        FtcChoiceMenu strategyMenu = new FtcChoiceMenu("Auto Strategies:", null, this);
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance:", strategyMenu, this);
        FtcChoiceMenu teamMenu     = new FtcChoiceMenu("Team:", allianceMenu, this);

        strategyMenu.addChoice("Shoot_Push_ParkCenter",      Field.AutoStrategy.SHOOT_PUSH_PARKCNTR,    allianceMenu);
        strategyMenu.addChoice("Shoot_Push_ParkCorner",      Field.AutoStrategy.SHOOT_PUSH_PARKCRNR,    allianceMenu);
        strategyMenu.addChoice("Shoot_ParkCenter",           Field.AutoStrategy.SHOOT_PARKCNTR,         allianceMenu);
        strategyMenu.addChoice("Shoot_ParkCorner",           Field.AutoStrategy.SHOOT_PARKCRNR,         allianceMenu);
        strategyMenu.addChoice("AngleShoot_Push_ParkCenter", Field.AutoStrategy.ANGSHOOT_PUSH_PARKCNTR, allianceMenu);
        strategyMenu.addChoice("AngleShoot_Push_ParkCorner", Field.AutoStrategy.ANGSHOOT_PUSH_PARKCRNR, allianceMenu);
        strategyMenu.addChoice("AngleShoot_ParkCenter",      Field.AutoStrategy.ANGSHOOT_PARKCNTR,      allianceMenu);
        strategyMenu.addChoice("AngleShoot_ParkCorner",      Field.AutoStrategy.ANGSHOOT_PARKCRNR,      allianceMenu);

        allianceMenu.addChoice("Red",  Field.Alliance.RED, teamMenu);
        allianceMenu.addChoice("Blue", Field.Alliance.BLUE, teamMenu);

        teamMenu.addChoice("Sonic", Team.SONIC);
        teamMenu.addChoice("Snowman", Team.SNOWMAN);

        //
        // Walk the menu tree starting with the strategy menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(strategyMenu);
        //
        // Set choices variables.
        //
        autoStrategy = (Field.AutoStrategy)strategyMenu.getCurrentChoiceObject();
        alliance = (Field.Alliance)allianceMenu.getCurrentChoiceObject();
        team = (Team)teamMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(0, "STRATEGY: %s", autoStrategy);
        dashboard.displayPrintf(1, "ALLIANCE: %s", alliance);
        dashboard.displayPrintf(2, "TEAM: %s", team);
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

    private final static double RGT_PUSH_POS = 0.2;
    private final static double LFT_PUSH_POS = 0.8;
    private final static double CTR_PUSH_POS = 0.5;

    //private final static double DEF_DRV_PWR  = 0.7;
    private final static double DEF_TRN_PWR  = 0.45;

    private final static double SHT_PWR_SONIC = 0.50;
    private final static double SHT_PWR_SNOWMAN = 0.50;
    private static double DEF_SHT_PWR = SHT_PWR_SONIC;
    private final static double DEF_SWP_PWR = 1.0;
    private final static double DEF_ELV_PWR = 0.5;

    private Segment[] pathSegs;

    private ShelbyBot   robot = new ShelbyBot();
    private ElapsedTime timer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private ImageTracker tracker;
    BeaconDetector bd = new BeaconDetector();
    private ButtonSide bSide = ButtonSide.UNKNOWN;

    private static Point2d curPos;
    private static double  curHdg;

    private static Field.AutoStrategy autoStrategy =
            Field.AutoStrategy.SHOOT_PARKCNTR;
    private static Field.Alliance alliance;

    private HalDashboard dashboard;

    private Team team = Team.SONIC;
    private double initHdg = 0.0;
}
