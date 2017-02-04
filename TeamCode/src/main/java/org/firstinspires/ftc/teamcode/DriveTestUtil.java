package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Point;

public class DriveTestUtil
{
    private Drivetrain drvTrn;
    private ShelbyBot robot;
    private LinearOpMode op;
    private ElapsedTime et;
    private DataLogger dl;

    public DriveTestUtil(ShelbyBot bot, LinearOpMode op, Drivetrain drvTrn, DataLogger dl)
    {
        this.robot = bot;
        this.op = op;
        this.drvTrn = drvTrn;
        this.dl = dl;
        et = new ElapsedTime();
    }

    void estAndLog()
    {
        drvTrn.estimatePosition();
        drvTrn.logData();
        robot.waitForTick(5);
    }

    /*
     Tests max speed.  DataLogger output can be used to calc speed from encoder counts
     and to determine accel/decel.
     Comparing RUN_USING_ENCODER vs RUN_WITHOUT_ENCODER could reveal PID
     max speed overhead.
     */
    public void doMaxSpeedTest(DcMotor.RunMode runMode)
    {
        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
        dl.addField("RUNNING MAX DRIVE SPEED TEST SWEEPER " + runMode.toString()); dl.newLine();
        drvTrn.stopAndReset();
        robot.leftMotor.setMode(runMode);
        robot.rightMotor.setMode(runMode);
        drvTrn.logData();
        et.reset();
        dl.addField("SETTING POWER 1.0"); dl.newLine();
        drvTrn.move(1.0);
        while(op.opModeIsActive() && et.seconds() < 2.0) { estAndLog(); }
        dl.addField("SETTING POWER 0.0"); dl.newLine();
        drvTrn.move(0.0);
        while(op.opModeIsActive() && et.seconds() < 3.0) { estAndLog(); }
        dl.addField("DONE MAX SPEED TEST " + runMode.toString()); dl.newLine();
        op.sleep(2000);
        robot.setDriveDir(ShelbyBot.DriveDir.PUSHER);
        dl.addField("RUNNING MAX DRIVE SPEED TEST PUSHER " + runMode.toString()); dl.newLine();
        robot.leftMotor.setMode(runMode);
        robot.rightMotor.setMode(runMode);
        et.reset();
        dl.addField("SETTING POWER 1.0"); dl.newLine();
        drvTrn.move(1.0);
        while(op.opModeIsActive() && et.seconds() < 2.0) { estAndLog(); }
        dl.addField("SETTING POWER 0.0"); dl.newLine();
        drvTrn.move(0.0);
        while(op.opModeIsActive() && et.seconds() < 3.0) { estAndLog(); }
        dl.addField("DONE MAX SPEED TEST " + runMode.toString()); dl.newLine();
        et.reset();
        while(op.opModeIsActive() && et.seconds() < 2.0) op.sleep(0);
        dl.addField("RUNNING MAX RT TURN SPEED TEST " + runMode.toString()); dl.newLine();
        et.reset();
        drvTrn.move(1.0, -1.0);
        while(op.opModeIsActive() && et.seconds() < 2.0) { estAndLog(); }
        dl.addField("SETTING POWER 0.0"); dl.newLine();
        drvTrn.move(0.0);
        while(op.opModeIsActive() && et.seconds() < 3.0) { estAndLog(); }
        dl.addField("DONE MAX RT TURN SPEED TEST " + runMode.toString()); dl.newLine();
        et.reset();
        while(op.opModeIsActive() && et.seconds() < 2.0) op.sleep(0);
        dl.addField("RUNNING MAX LT TURN SPEED TEST " + runMode.toString()); dl.newLine();
        et.reset();
        drvTrn.move(-1.0, 1.0);
        while(op.opModeIsActive() && et.seconds() < 2.0) { estAndLog(); }
        dl.addField("SETTING POWER 0.0"); dl.newLine();
        drvTrn.move(0.0);
        while(op.opModeIsActive() && et.seconds() < 3.0) { estAndLog(); }
        dl.addField("DONE MAX LT TURN SPEED TEST " + runMode.toString()); dl.newLine();
    }

    public void doMinSpeedTest()
    {
        double spdStep = 0.01;
        double spdDur  = 1.0;
        int    MOVE_THRESH = 10;
        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
        dl.addField("RUNNING MIN SPEED TEST"); dl.newLine();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        for(double spd = spdStep; spd < 0.15; spd+=spdStep )
        {
            dl.addField("SETTING POWER " + spd); dl.newLine();
            drvTrn.stopAndReset();
            et.reset();
            drvTrn.move(spd);
            while(op.opModeIsActive() && et.seconds() < spdDur)
            {
                estAndLog();
            }
            drvTrn.move(0);
            boolean moved = true;
            int l = robot.leftMotor.getCurrentPosition();
            int r = robot.rightMotor.getCurrentPosition();
            if(l < MOVE_THRESH || r < MOVE_THRESH) moved = false;
            dl.addField("DRVSPD"); dl.addField("", spd); dl.addField(l); dl.addField(r);
            dl.addField(moved); dl.newLine();
        }
    }

    public void driveDist(double dist, double pwr, boolean useAnd)
    {
        int counts = drvTrn.distanceToCounts(dist);
        Point2d strtPt = new Point2d(0,0);
        Point2d tgtPt = new Point2d(dist, 0);
        drvTrn.setCurrPt(strtPt);
        drvTrn.setBusyAnd(useAnd);
        drvTrn.stopAndReset();
        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
        dl.addField("TEST DRIVE DISTANCE at power");
        dl.addField("", dist); dl.addField("", pwr); dl.newLine();
        drvTrn.driveToPointLinear(tgtPt, pwr, Drivetrain.Direction.FORWARD);
        et.reset();
        while(op.opModeIsActive() && et.seconds() < 5.0) { }
        robot.invertDriveDir();
        drvTrn.driveToPointLinear(strtPt, pwr, Drivetrain.Direction.FORWARD);
    }
}
