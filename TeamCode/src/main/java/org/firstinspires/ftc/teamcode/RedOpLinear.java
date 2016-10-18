
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@SuppressWarnings("unused")
@Autonomous(name="RedOpLinear", group="Auton")
//@Disabled
public class RedOpLinear extends LinearOpMode {

    public RedOpLinear()
    {
        super();
        instance = this;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        setup();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        timer.reset();

        for(int i = 0; i< pathSegs.length; ++i)
        {
            doMove(pathSegs[i]);
            if(i < turns.length)
            {
                doTurn(turns[i]);
                DbgLog.msg("Planned pos: %s %s",
                        pathSegs[i].getTgtPt(),
                        pathSegs[i+1].getFieldHeading());
                findSensedLoc();
            }
        }

        drvTrn.stopAndReset();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    static LinearOpMode getInstance()
    {
        return instance;
    }

    private void setup()
    {
        String className = this.getClass().getName();
        String first3 = className.substring(0,2);
        if(first3.equals("Blu"))  alliance = Field.Alliance.BLUE;
        else                      alliance = Field.Alliance.RED;

        robot.init(hardwareMap);
        drvTrn.init(robot.leftMotor, robot.rightMotor);

        Points pts = new Points();
        pathSegs = pts.getSegments(alliance);
        turns    = pts.getTurns(alliance);

        DbgLog.msg("SJH ROUTE: \n" + pts.toString());

        Point2d currPoint = pathSegs[0].getStrtPt();
        drvTrn.setCurrPt(currPoint);

        timer.reset();
        DbgLog.msg("SJH Start %s. Time: %6.3f", currPoint, timer.time());

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path", "Start at %s", currPoint);
        telemetry.update();
    }

    private void doMove(Segment seg) throws InterruptedException
    {
        String  snm = seg.getName();
        Point2d spt = seg.getStrtPt();
        Point2d ept = seg.getTgtPt();
        double  fhd = seg.getFieldHeading();
        Segment.SegDir dir = seg.getDir();
        DbgLog.msg("SJH: Drive %s %s %s %6.2f %s",
                snm, spt, ept, fhd, dir);

        telemetry.addData("Path", "%s %s - %s %6.2f %s",
                snm, spt, ept, fhd, dir);
        telemetry.update();

        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;
        if (dir == Segment.SegDir.REVERSE) ddir = Drivetrain.Direction.REVERSE;
        timer.reset();
        Point2d pt = seg.getTgtPt();
        drvTrn.driveToPointLinear(pt, DEF_DRV_PWR, ddir);
        DbgLog.msg("SJH Completed move %s. Time: %6.3f", seg.getName(), timer.time());
    }

    private void doTurn(double angle) throws InterruptedException
    {
        DbgLog.msg("SJH: Turn %5.2f", angle);
        timer.reset();
        drvTrn.ctrTurnLinear(angle,DEF_TRN_PWR);
        DbgLog.msg("SJH Completed turn %5.2f. Time: %6.3f", angle, timer.time());
    }

    private void findSensedLoc() throws InterruptedException
    {
        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        tracker.setActive(true);
        Thread.sleep(200);
        Point2d sensedBotPos = null;
        double  sensedFldHdg = pathSegs[0].getFieldHeading();
        while(sensedBotPos == null && itimer.milliseconds() < 200)
        {
            tracker.updateRobotLocationInfo();
            sensedBotPos = tracker.getSensedPosition();
            sensedFldHdg = tracker.getSensedFldHeading();
        }
        tracker.setActive(false);

        if ( sensedBotPos != null )
            DbgLog.msg("Image based location: %s %5.2f", sensedBotPos, sensedFldHdg);
    }

    private final static double DEF_DRV_PWR = 0.7;
    private final static double DEF_TRN_PWR = 0.5;

    private Segment[] pathSegs;
    private double[]  turns;

    private ShelbyBot   robot = new ShelbyBot();
    private ElapsedTime timer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private ImageTracker tracker = new ImageTracker();

    private static Field.Alliance alliance;
    private static LinearOpMode instance = null;
}
