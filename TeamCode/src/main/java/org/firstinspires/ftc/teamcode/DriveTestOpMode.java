package org.firstinspires.ftc.teamcode;

import android.widget.TextView;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.Date;

import hallib.HalDashboard;

@Autonomous(name="DriveTestOpMode", group="Auton")
public class DriveTestOpMode extends LinearOpMode
{

    public void initRobot()
    {
        telemetry.addData("_","PLEASE WAIT - STARTING");
        telemetry.update();
        dashboard = HalDashboard.createInstance(telemetry);

        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);
        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));
        setup();
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

        dtu = new DriveTestUtil(robot, this, drvTrn, dl);

        dashboard.displayPrintf(0, "GYRO CALIBRATING DO NOT TOUCH OR START");

        robot.turnColorOff();

        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);

        if (robot.leftMotor != null &&
            robot.rightMotor != null &&
            robot.gyro != null)
        {
            int lms = robot.leftMotor.getMaxSpeed();
            int rms = robot.rightMotor.getMaxSpeed();
            DbgLog.msg("SJH: MaxSpeeds %d %d", lms, rms);

            gyroReady = robot.calibrateGyro();
        }

        if (gyroReady)
            dashboard.displayPrintf(0, "GYRO CALIBATED!!");

        drvTrn.setCurrPt(new Point2d(0, 0));

        drvTrn.setStartHdg(0);
        robot.setInitHdg(0);
    }

    public void runOpMode()
    {
        initRobot();
        while(!isStarted())
        {
            dashboard.displayPrintf(1, "HDG: " + robot.getGyroFhdg());
            dashboard.displayPrintf(2, "LENC: " + robot.leftMotor.getCurrentPosition());
            dashboard.displayPrintf(3, "RENC: " + robot.rightMotor.getCurrentPosition());
        }
        waitForStart();
        startMode();
        stopMode();
    }

    public void startMode()
    {
        dashboard.clearDisplay();
        do_main_loop();
    }

    boolean doSpeedTest = false;
    boolean doMaxSpeedTest = false;
    boolean doMinSpeedTest = false;
    boolean doFindBestDriveSpeed = false; //true;
    boolean doFindBestEncTurnSpeed = false;
    boolean doFindBestGyroTurnSpeed = false;
    boolean doDriveDist = false;
    boolean doTurnAngle = false;
    boolean doMotionProfile = true;

    boolean useAnd = true;
    boolean rampUp = true;
    boolean rampDown = true;
    boolean stopIndiv = false;

    public void do_main_loop()
    {
        dl.addField("useAnd"); dl.addField(useAnd); dl.newLine();
        dl.addField("rampUp"); dl.addField(rampUp); dl.newLine();
        dl.addField("rampDown"); dl.addField(rampDown); dl.newLine();
        dl.addField("stopIndiv"); dl.addField(stopIndiv); dl.newLine();

        drvTrn.setLogOverrun(true);
        drvTrn.setBusyAnd(useAnd);
        drvTrn.setRampDown(rampDown);
        drvTrn.setRampUp(rampUp);
        drvTrn.setStopIndividualMotorWhenNotBusy(stopIndiv);

        DbgLog.msg("SJH: Starting test do_main_loop");
        robot.gyro.resetZAxisIntegrator();
        sleep(100);

        //Test speed and accel/decel for various setPower settings
        double tHdg = 0.0;
        if(doSpeedTest)
        {
            dl.addField("doSpeedTest"); dl.newLine();
            for (double s = 0.1; s <= 1.0; s += 0.1)
            {
                DbgLog.msg("SJH: Test speed " + s);
                drvTrn.logData(true, "TURNING");
                drvTrn.ctrTurnLinear(tHdg - robot.getGyroFhdg(), 0.4);
                drvTrn.ctrTurnToHeading(tHdg, 0.2);
                tHdg += 180.0;
                if (tHdg >= 360) tHdg = 0.0;
                drvTrn.stopAndReset();
                sleep(100);
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(100);
                dtu.doSpeedTest(DcMotor.RunMode.RUN_USING_ENCODER, s, 0.5, 0.5, 0.5);
                sleep(1000);
            }
        }

        //Find optimal drive speeds
        if(doFindBestDriveSpeed)
        {
            dl.addField("doFindBestDriveSpeed"); dl.newLine();
            dtu.findBestDriveSpeed();
        }

        //Drive motion profile test
        if(doMotionProfile)
        {
            double d = 24.0;
            double s = 0.7;
            dl.addField("doMotionProfile_"+d+"_"+s); dl.newLine();
            dtu.doMotionProfile(d, s);
        }

        //Find optimal encoder speed
        if(doFindBestEncTurnSpeed)
        {
            dl.addField("doFindBestEncTurnSpeed"); dl.newLine();
            dtu.findBestEncTurnSpeed();
        }

        //Find optimal gyro speed/gain
        if(doFindBestGyroTurnSpeed)
        {
            dl.addField("doFindBestGyroTurnSpeed"); dl.newLine();
            dtu.findBestGyroTurnSpeedGain();
        }

        //Test some common distance and turns from auton
        if(doDriveDist)
        {
            double dist[] = {24.0, 48.0};
            for (int i = 0 ; i < dist.length; i++)
            {
                drvTrn.setCurrPt(new Point2d(0,0));
                Point2d tPt = new Point2d(dist[i], 0.0);
                dl.addField("doDriveDist " + dist);
                drvTrn.driveToPointLinear(tPt, 0.8, Drivetrain.Direction.FORWARD);
            }
        }

        if(doTurnAngle)
        {
            double angle = -90.0;
            drvTrn.ctrTurnLinear(angle - robot.getGyroFhdg(), 0.4);
            dl.addField("doTurnAngle"); dl.newLine();
            drvTrn.ctrTurnToHeading(angle, 0.4);
        }


        if(doMaxSpeedTest)
        {
            dl.addField("doMaxSpeedTest"); dl.newLine();
            dtu.doMaxSpeedTest(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sleep(2000);
            dtu.doMaxSpeedTest(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(1000);
        }

        if(doMinSpeedTest)
        {
            dl.addField("doMinSpeedTest"); dl.newLine();
            dtu.doMinSpeedTest();
            sleep(1000);
        }
//        dtu.driveDist(12.0, 0.2, false);
//        sleep(5000);
//        dtu.driveDist(12.0, 0.2, true);
        ElapsedTime endTimer = new ElapsedTime();
        while (endTimer.seconds() < 5)
        {
            dashboard.displayPrintf(1, "HDG: " + robot.getGyroFhdg());
            dashboard.displayPrintf(2, "LENC: " + robot.leftMotor.getCurrentPosition());
            dashboard.displayPrintf(3, "RENC: " + robot.rightMotor.getCurrentPosition());
        }
    }

    public void stopMode()
    {
        if(drvTrn != null) drvTrn.stopAndReset();
        dl.closeDataLogger();
    }

    private void setupLogger()
    {
        if (logData)
        {
            Date day = new Date();
            dl = new DataLogger("driveTest");
            dl.addField("NOTE");
            dl.addField("Frame");
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

    private static HalDashboard dashboard = null;
    private boolean gyroReady;
    private ShelbyBot   robot = new ShelbyBot();
    private Drivetrain drvTrn = new Drivetrain();
    private DataLogger dl;
    private DriveTestUtil dtu;
    private boolean logData = true;
}
