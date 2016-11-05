
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Vector;

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
//        instance = this;
    }

    @Override
    public void initRobot()
    {
        dashboard = getDashboard();
        dashboard.displayPrintf(2, "STATE: %s", "INITIALIZING");

        setup();
        firstPass = true;
    }

    @Override
    public void runPeriodic(double elapsedTime)
    {
        dashboard.displayPrintf(6, "GHDG: %d",
                robot.gyro.getIntegratedZValue());

        if(firstPass)
        {
            try
            {
                for (int i = 0; i < pathSegs.length; ++i)
                {
                    doMove(pathSegs[i], speeds.get(i), dirs.get(i));
                    if (i < turns.length)
                    {
                        //doTurn(turns[i]);
                        doTurn(pathSegs[i+1]);
                        DbgLog.msg("SJH Planned pos: %s %s",
                                pathSegs[i].getTgtPt(),
                                pathSegs[i + 1].getFieldHeading());
                    }
                    switch (actions.get(i))
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
                            do_findBeaconOrder(true);
                            break;
                        case RST_PUSHER:
                            robot.pusher.setPosition(RGT_PUSH_POS);
                            break;
                        case NOTHING:
                            break;
                    }
                }
            } catch (InterruptedException e)
            {
                DbgLog.error("SJH Interruped");
            }
        }
        firstPass = false;
    }

    @Override
    public void startMode()
    {
        robot.gyro.resetZAxisIntegrator();
    }

    @Override
    public void stopMode()
    {
        //drvTrn.stopAndReset();
        //dashboard.displayPrintf(2, "STATE: %s", "PATH COMPLETE");
    }

    private void setup()
    {
        robot.init(hardwareMap);

        doMenus();
//CAMERA STUFF create camera and callback then call ftca.init
//        FtcRobotControllerActivity ftca = (FtcRobotControllerActivity) hardwareMap.appContext;
//        LinearLayout previewLayout;
//        previewLayout = (LinearLayout) ftca.findViewById(com.qualcomm.ftcrobotcontroller
//                                                                 .R.id.cameraMonitorViewId);
//        Camera camera = null;
//        Camera.PreviewCallback previewCallback = null;
//        //need to create preview in UI thread need to get preview?
//        CameraPreview preview = new CameraPreview(ftca, camera, previewCallback);
//END CAMERA STUFF

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
        turns    = pts.getTurns(alliance);
        actions  = pts.getActions();
        dirs     = pts.getSegDirs();
        speeds   = pts.getSegSpeeds();

        double initHdg = pathSegs[0].getFieldHeading();

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

        //FtcRobotControllerActivity act =
        //        (FtcRobotControllerActivity)hardwareMap.appContext;

        //ocah = new OpenCvActivityHelper(act);
        //ocah.addCallback(bd);
    }

    private void doMove(Segment seg, double speed, Segment.SegDir dir)
            throws InterruptedException
    {
        String  snm = seg.getName();
        Point2d spt = seg.getStrtPt();
        Point2d ept = seg.getTgtPt();
        double  fhd = seg.getFieldHeading();
        DbgLog.msg("SJH: Drive %s %s %s %6.2f %s",
                snm, spt, ept, fhd, dir);

        dashboard.displayPrintf(2, "STATE: %s %s %s - %s %6.2f %s",
                "DRIVE", snm, spt, ept, fhd, dir);

        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;
        if (dir == Segment.SegDir.REVERSE) ddir = Drivetrain.Direction.REVERSE;
        timer.reset();
        Point2d pt = seg.getTgtPt();
        drvTrn.driveToPointLinear(pt, speed, ddir);
        DbgLog.msg("SJH Completed move %s. Time: %6.3f", seg.getName(), timer.time());
    }

    private void doTurn(double angle) throws InterruptedException
    {
        DbgLog.msg("SJH: Turn %5.2f", angle);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        //drvTrn.ctrTurnLinear(angle,DEF_TRN_PWR);
        drvTrn.ctrTurnLinearGyro(angle,DEF_TRN_PWR);
        DbgLog.msg("SJH Completed turn %5.2f. Time: %6.3f", angle, timer.time());
    }

    private void doTurn(Segment seg)
    {
        double hdg = seg.getFieldHeading();
        if(seg.getDir() == Segment.SegDir.REVERSE)
            hdg += 180;
        DbgLog.msg("SJH: TurnToHdg %5.2f", hdg);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "HDG", hdg);
        timer.reset();
        drvTrn.ctrTurnToHeading(hdg, DEF_TRN_PWR);
        DbgLog.msg("SJH Completed turnHdg %5.2f. Time: %6.3f", hdg, timer.time());
    }

    private boolean findSensedLoc() throws InterruptedException
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
            Thread.sleep(50);
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

    @SuppressWarnings("ConstantConditions")
    private void do_findBeaconOrder(boolean push) throws InterruptedException
    {
        DbgLog.msg("SJH: FIND BEACON ORDER!!!");
        dashboard.displayPrintf(2, "STATE: %s", "BEACON FIND");
        int timeout = 1000;
        BeaconFinder.LightOrder ord = BeaconFinder.LightOrder.UNKNOWN;
        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        //ocah.attach();
        while (ord == BeaconFinder.LightOrder.UNKNOWN &&
               itimer.milliseconds() < timeout)
        {
            //ord = bd.getLightOrder();
            Thread.sleep(50);
        }
        //ocah.stop();

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

        //TESTING ONLY - remove when beacon finder is working
        if(bSide == ButtonSide.UNKNOWN)
            bSide = ButtonSide.LEFT;
        else if (bSide == ButtonSide.LEFT)
            bSide = ButtonSide.RIGHT;
        DbgLog.msg("SJH: Gonna push button " + bSide);
        dashboard.displayPrintf(5, "BUTTON: %s", bSide);

        if(push)
        {
            do_pushButton(bSide);
        }
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

        strategyMenu.addChoice("Shoot_Push_ParkCenter",      Field.AutoStrategy.SHOOT_PUSH_PARKCNTR,    allianceMenu);
        strategyMenu.addChoice("Shoot_Push_ParkCorner",      Field.AutoStrategy.SHOOT_PUSH_PARKCRNR,    allianceMenu);
        strategyMenu.addChoice("Shoot_ParkCenter",           Field.AutoStrategy.SHOOT_PARKCNTR,         allianceMenu);
        strategyMenu.addChoice("Shoot_ParkCorner",           Field.AutoStrategy.SHOOT_PARKCRNR,         allianceMenu);
        strategyMenu.addChoice("AngleShoot_Push_ParkCenter", Field.AutoStrategy.ANGSHOOT_PUSH_PARKCNTR, allianceMenu);
        strategyMenu.addChoice("AngleShoot_Push_ParkCorner", Field.AutoStrategy.ANGSHOOT_PUSH_PARKCRNR, allianceMenu);
        strategyMenu.addChoice("AngleShoot_ParkCenter",      Field.AutoStrategy.SHOOT_PARKCNTR,         allianceMenu);
        strategyMenu.addChoice("AngleShoot_ParkCorner",      Field.AutoStrategy.SHOOT_PARKCRNR,         allianceMenu);

        allianceMenu.addChoice("Red",  Field.Alliance.RED);
        allianceMenu.addChoice("Blue", Field.Alliance.BLUE);

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

        dashboard.displayPrintf(0, "Auto Strategy: %s", autoStrategy);
        dashboard.displayPrintf(1, "Alliance: %s", alliance);
    }

    private enum ButtonSide
    {
        UNKNOWN,
        LEFT,
        RIGHT
    }

    private final static double RGT_PUSH_POS = 0.2;
    private final static double LFT_PUSH_POS = 0.8;

    //private final static double DEF_DRV_PWR  = 0.7;
    private final static double DEF_TRN_PWR  = 0.3;

    private final static double DEF_SHT_PWR = 0.55;
    private final static double DEF_SWP_PWR = 1.0;
    private final static double DEF_ELV_PWR = 0.5;

    private Segment[] pathSegs;
    private double[]  turns;
    private Vector<Points.Action> actions;
    private Vector<Segment.SegDir> dirs;
    private Vector<Double> speeds;

    private ShelbyBot   robot = new ShelbyBot();
    private ElapsedTime timer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private ImageTracker tracker = new ImageTracker();
    //BeaconDetector bd = new BeaconDetector();
    //private OpenCvActivityHelper ocah;
    private ButtonSide bSide = ButtonSide.UNKNOWN;

//    private static LinearOpMode instance = null;

    private static Point2d curPos;
    private static double  curHdg;

    private static Field.AutoStrategy autoStrategy =
            Field.AutoStrategy.SHOOT_PARKCNTR;
    private static Field.Alliance alliance;

    private HalDashboard dashboard;

    private boolean firstPass;
}
