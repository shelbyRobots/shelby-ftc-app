package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.RobotLog;
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
        drvTrn.setCurValues();
        drvTrn.logData();
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
        op.sleep(50);
        robot.leftMotor.setMode(runMode);
        robot.rightMotor.setMode(runMode);
        drvTrn.logData();
        et.reset();
        dl.addField("SETTING POWER 1.0"); dl.newLine();
        drvTrn.move(1.0);
        while(op.opModeIsActive() && et.seconds() < 2.0) { estAndLog(); drvTrn.waitForTick(10); }
        dl.addField("SETTING POWER 0.0"); dl.newLine();
        drvTrn.move(0.0);
        while(op.opModeIsActive() && et.seconds() < 3.0) { estAndLog(); drvTrn.waitForTick(10); }
        dl.addField("DONE MAX SPEED TEST " + runMode.toString()); dl.newLine();
        op.sleep(2000);
        robot.setDriveDir(ShelbyBot.DriveDir.PUSHER);
        dl.addField("RUNNING MAX DRIVE SPEED TEST PUSHER " + runMode.toString()); dl.newLine();
        robot.leftMotor.setMode(runMode);
        robot.rightMotor.setMode(runMode);
        et.reset();
        dl.addField("SETTING POWER 1.0"); dl.newLine();
        drvTrn.move(1.0);
        while(op.opModeIsActive() && et.seconds() < 2.0) { estAndLog(); drvTrn.waitForTick(10);}
        dl.addField("SETTING POWER 0.0"); dl.newLine();
        drvTrn.move(0.0);
        while(op.opModeIsActive() && et.seconds() < 3.0) { estAndLog(); drvTrn.waitForTick(10);}
        dl.addField("DONE MAX SPEED TEST " + runMode.toString()); dl.newLine();
        et.reset();
        while(op.opModeIsActive() && et.seconds() < 2.0) op.sleep(0);
        dl.addField("RUNNING MAX RT TURN SPEED TEST " + runMode.toString()); dl.newLine();
        et.reset();
        drvTrn.move(1.0, -1.0);
        while(op.opModeIsActive() && et.seconds() < 2.0) { estAndLog(); drvTrn.waitForTick(10);}
        dl.addField("SETTING POWER 0.0"); dl.newLine();
        drvTrn.move(0.0);
        while(op.opModeIsActive() && et.seconds() < 3.0) { estAndLog(); drvTrn.waitForTick(10);}
        dl.addField("DONE MAX RT TURN SPEED TEST " + runMode.toString()); dl.newLine();
        et.reset();
        while(op.opModeIsActive() && et.seconds() < 2.0) op.sleep(0);
        dl.addField("RUNNING MAX LT TURN SPEED TEST " + runMode.toString()); dl.newLine();
        et.reset();
        drvTrn.move(-1.0, 1.0);
        while(op.opModeIsActive() && et.seconds() < 2.0) { estAndLog(); drvTrn.waitForTick(10);}
        dl.addField("SETTING POWER 0.0"); dl.newLine();
        drvTrn.move(0.0);
        while(op.opModeIsActive() && et.seconds() < 3.0) { estAndLog(); drvTrn.waitForTick(10);}
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
                drvTrn.waitForTick(10);
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

    public void doSpeedTest(DcMotor.RunMode runMode, double spd,
                            double atime, double rtime, double dtime)
    {
        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
        dl.addField("RUNNING MAX DRIVE SPEED TEST SWEEPER " + runMode.toString());
        dl.newLine();
        drvTrn.logData(true, "SETTING POWER " + spd);
        et.reset();
        dl.resetTime();
        drvTrn.moveInit(spd, spd);
        while (op.opModeIsActive() && et.seconds() < atime)
        {
            estAndLog();
            drvTrn.waitForTick(10);
        }
        drvTrn.logData(true, "DONE ACCEL");
        et.reset();
        double t0 = et.seconds();
        int p0 = (robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition())/2;
        while (op.opModeIsActive() && et.seconds() < rtime)
        {
            estAndLog();
            drvTrn.waitForTick(10);
        }
        double t1 = et.seconds();
        int p1 = (robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition())/2;
        double rate = (p1-p0)/(t1-t0);
        drvTrn.logData(true, "SPEED FOR POWER " + spd + "=" + rate);
        drvTrn.logData(true, "SETTING POWER 0.0");
        drvTrn.stopMotion();
        et.reset();
        while (op.opModeIsActive() && et.seconds() < dtime)
        {
            estAndLog();
            drvTrn.waitForTick(10);
        }
    }

    public void driveDist(double dist, double pwr)
    {
        int counts = drvTrn.distanceToCounts(dist);
        Point2d strtPt = new Point2d(0,0);
        Point2d tgtPt = new Point2d(dist, 0);
        drvTrn.setCurrPt(strtPt);
        drvTrn.stopAndReset();
        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
        dl.addField("TEST DRIVE DISTANCE at power");
        dl.addField("", dist); dl.addField("", pwr); dl.newLine();
        op.sleep(200);
        dl.resetTime();
        drvTrn.driveToPointLinear(tgtPt, pwr, Drivetrain.Direction.FORWARD);
        et.reset();
        op.sleep(2000);
        robot.invertDriveDir();
        drvTrn.driveToPointLinear(strtPt, pwr, Drivetrain.Direction.FORWARD);
    }

    public void findBestDriveSpeed()
    {
        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
        double distances[] = {48}; //{24, 43};
        double tHdg = 0.0;
        op.sleep(200);
        for(int d=0; d < distances.length; d++)
        {
            double dist = distances[d];
            for (double spd = 0.25; spd <= 0.75; spd += 0.05)
            {
                double cHdg = robot.getGyroFhdg();
                drvTrn.ctrTurnLinear(tHdg-cHdg, 0.4);
                drvTrn.ctrTurnToHeading(tHdg, 0.2);
                tHdg += 180.0;
                if (tHdg >= 360) tHdg = 0.0;
                drvTrn.stopAndReset();
                drvTrn.logData(true, "START SPD OPT " + spd + " " + dist);
                dl.resetTime();
                drvTrn.driveDistanceLinear(dist, spd, Drivetrain.Direction.FORWARD);
            }
        }
    }

    public void findBestEncTurnSpeed()
    {
        double turnAngles[] = {-29, 29, -90, 90, -119, 119};
        double tHdg = 0.0;
        drvTrn.ctrTurnToHeading(tHdg, 0.2);
        for( int a = 0; a < turnAngles.length; a++)
        {
            double angle = turnAngles[a];
            for(double spd = 0.15; spd <= 0.75; spd+=0.1)
            {
                dl.resetTime();
                dl.addField("START ENC SPD OPT " + angle + " " + spd);
                drvTrn.ctrTurnLinear(angle, spd);
                op.sleep(100);
            }
        }
    }

    public void findBestGyroTurnSpeedGain()
    {
        int turnAngles[] = {3, 10};
        double tHdg = 0.0;
        drvTrn.ctrTurnToHeading(tHdg, 0.2);
        for( int a = 0; a < turnAngles.length; a++)
        {
            int angle = turnAngles[a];
            for(double spd = 0.1; spd <= 0.5; spd+=0.1)
            {
                double startGain = 0.01;
                double endGain = 0.04;
                double gainStep = 0.005;
                for(double gain = startGain; gain < endGain; gain += gainStep)
                {
                    dl.addField("START GYRO SPD OPT " + angle + " " + spd + " " + gain);
                    int curH = robot.getGyroFhdg();
                    int trgH = curH + angle;
                    drvTrn.Kp_GyroTurn = gain;
                    drvTrn.ctrTurnToHeading(curH, spd);
                }
            }
        }
    }

    public void doMotionProfile(double dist, double spd)
    {
        ElapsedTime mpTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double accEndPos = 400;
        int tgtPos = drvTrn.distanceToCounts(dist);
        int crzEndPos = tgtPos - 280;

        boolean filter = true;

        int f1Ticks = 12;
        int f2Ticks = 6;

        double f1spd = 0;
        double f2spd = 0;

        int frm = 0;

        int nFiltFrms = Math.max(f1Ticks, f2Ticks);

        double aspds[] = new double[nFiltFrms];
        double dspds[] = new double[nFiltFrms];

        RobotLog.ii("SJH", "Accel speeds");
        while(frm < nFiltFrms)
        {
            f1spd = (double)Math.min(frm+1, f1Ticks)/f1Ticks * spd;
            f2spd = (double)Math.min(frm+1, f2Ticks)/f2Ticks * f1spd;
            aspds[frm] = f2spd;
            RobotLog.ii("SJH", "aspds["+frm+"]="+aspds[frm]);
            frm++;
        }

        RobotLog.ii("SJH", "Decel speeds");
        frm = 0;
        while(frm < nFiltFrms)
        {
            f1spd = (double)Math.min(frm+1, f1Ticks)/f1Ticks * spd;
            f2spd = (double)Math.min(frm+1, f2Ticks)/f2Ticks * f1spd;
            dspds[frm] = spd - f2spd;
            RobotLog.ii("SJH", "dspds["+frm+"]="+dspds[frm]);
            frm++;
        }

        frm = 0;
        drvTrn.waitForTick(10);
        while(op.opModeIsActive() && robot.leftMotor.getCurrentPosition() < accEndPos)
        {
            estAndLog();
            double cspd = spd;
            if(filter && frm < nFiltFrms) cspd = aspds[frm];
            drvTrn.move(cspd);
            drvTrn.waitForTick(10);
            frm++;
        }

        drvTrn.move(spd);

        while(op.opModeIsActive() && robot.leftMotor.getCurrentPosition() < crzEndPos)
        {
            estAndLog();
            drvTrn.waitForTick(10);
        }

        frm = 0;
        while(op.opModeIsActive() && robot.leftMotor.getCurrentPosition() < tgtPos)
        {
            estAndLog();
            double cspd = 0;
            if(filter && frm < nFiltFrms) cspd = dspds[frm];
            drvTrn.move(cspd);
            drvTrn.waitForTick(10);
            frm++;
        }

        drvTrn.stopMotion();
    }

    public void doDoubleCurveTurn(double sideDist, double spd)
    {
        double theta = Math.toDegrees(Math.acos(1 - sideDist/ShelbyBot.BOT_WIDTH));
        drvTrn.logData(true, "S turn " + theta + " " + spd + " " + sideDist);
        drvTrn.turn(theta, spd,   ShelbyBot.BOT_WIDTH/2);
        drvTrn.turn(-theta,spd, -ShelbyBot.BOT_WIDTH/2);
        op.sleep(1000);
        drvTrn.turn(90, spd, ShelbyBot.BOT_WIDTH);
    }
}
