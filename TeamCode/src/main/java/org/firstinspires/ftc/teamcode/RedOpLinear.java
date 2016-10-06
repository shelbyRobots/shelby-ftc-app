
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="RedOpLinear", group="Auton")
//@Disabled
public class RedOpLinear extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException
    {
        setup();
        idle();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        timer.reset();

        for(int i = 0; i< pathSegs.length; ++i)
        {
            doMove(pathSegs[i]);
            if(i < turns.length)
                doTurn(turns[i]);
        }

        Drivetrain.stopAndReset();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    LinearOpMode getInstance()
    {
        instance = this;
        return instance;
    }

    private void setup()
    {
        String className = this.getClass().getName();
        String first3 = className.substring(0,2);
        if(first3.equals("Blu"))  alliance = Field.Alliance.BLUE;
        else                      alliance = Field.Alliance.RED;

        robot.init(hardwareMap);
        Drivetrain.init(robot.leftMotor, robot.rightMotor);

        Points pts = new Points();
        pathSegs = pts.getSegments(alliance);
        turns    = pts.getTurns(alliance);

        Point2d currPoint = pathSegs[0].getStrtPt();
        Drivetrain.setCurrPt(currPoint);

        timer.reset();
        DbgLog.msg("SJH Start %s. Time: %6.3f", currPoint, timer.time());

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path", "Start at %s", currPoint);
        telemetry.update();
    }

    private void doMove(Segment seg)
    {
        DbgLog.msg("SJH: Drive %s %s %s %6.2f %s",seg.getName(),
                seg.getStrtPt(), seg.getTgtPt(), seg.getFieldHeading(), seg.getDir());

        telemetry.addData("Path", "%s %s - %s %6.2f %s", seg.getName(),
                seg.getStrtPt(), seg.getTgtPt(), seg.getFieldHeading(), seg.getDir());
        telemetry.update();

        timer.reset();
        Point2d pt = seg.getTgtPt();
        Drivetrain.driveToPointLinear(pt, DEF_DRV_PWR, Drivetrain.Direction.FORWARD);
        Drivetrain.setCurrPt(pt);
        DbgLog.msg("SJH Completed move %s. Time: %6.3f", seg.getName(), timer.time());
    }

    private void doTurn(double angle)
    {
        DbgLog.msg("SJH: Turn %5.2f", angle);
        timer.reset();
        Drivetrain.ctrTurnLinear(angle,DEF_TRN_PWR);
        DbgLog.msg("SJH Completed turn %5.2f. Time: %6.3f", angle, timer.time());
    }

    private final static double DEF_DRV_PWR = 0.7;
    private final static double DEF_TRN_PWR = 0.5;

    private Segment[] pathSegs;
    private double[]  turns;

    private ShelbyBot   robot = new ShelbyBot();
    private ElapsedTime timer = new ElapsedTime();

    private static Field.Alliance alliance;
    private static LinearOpMode instance;
}
