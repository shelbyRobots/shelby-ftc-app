package org.firstinspires.ftc.teamcode;

import android.widget.TextView;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.Date;
import java.util.HashMap;
import java.util.Map;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import hallib.HalDashboard;

@Autonomous(name="DriveTestOpMode", group="Auton")
public class DriveTestOpMode extends LinearOpMode implements FtcMenu.MenuButtons
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

    Map<String, Boolean> testMap = new HashMap<>();
    String testToRun = "doSpeedTest";
    String tests[] =
            {
                    "doSpeedTest",
                    "doMaxSpeedTest",
                    "doMinSpeedTest",
                    "doFindBestDriveSpeed",
                    "doFindBestEncTurnSpeed",
                    "doFindBestGyroTurnSpeed",
                    "doDriveDist",
                    "doTurnAngle",
                    "doMotionProfile",
                    "doSturnTest"
            };
    
    private void setup()
    {
//        for(String str : tests)
//        {
//            testMap.put(str, str.equals(testToRun));
//        }
        
        dashboard.displayPrintf(2, "STATE: %s", "INITIALIZING - PLEASE WAIT FOR MENU");
        RobotLog.ii("SJH", "SETUP");
        hardwareMap.logDevices();

        robot.init(this);

        drvTrn.init(robot);
        drvTrn.setOpMode(this);

        doMenus();

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
        robot.turnColorOn();
        while(!isStarted())
        {
            dashboard.displayPrintf(1, "HDG: " + robot.getGyroFhdg());
            dashboard.displayPrintf(2, "LENC: " + robot.leftMotor.getCurrentPosition());
            dashboard.displayPrintf(3, "RENC: " + robot.rightMotor.getCurrentPosition());
            dashboard.displayPrintf(4, "R " + robot.colorSensor.red());
            dashboard.displayPrintf(5, "G " + robot.colorSensor.green());
            dashboard.displayPrintf(6, "B " + robot.colorSensor.blue());
        }
        robot.turnColorOff();
        waitForStart();
        startMode();
        stopMode();
    }

    public void startMode()
    {
        dashboard.clearDisplay();
        do_main_loop();
    }

    boolean useAnd = true;
    boolean rampUp = false;
    boolean rampDown = false;
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
        drvTrn.setUseSpeedThreads(false);
        drvTrn.setGangMotors(false);

        RobotLog.ii("SJH", "Starting test do_main_loop");
        robot.gyro.resetZAxisIntegrator();
        sleep(100);

        //Test speed and accel/decel for various setPower settings
        double tHdg = 0.0;
        if(testMap.get("doSpeedTest"))
        {
            dl.addField("doSpeedTest"); dl.newLine();
            for (double s = 0.1; s <= 1.0; s += 0.1)
            {
                RobotLog.ii("SJH", "Test speed " + s);
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
        if(testMap.get("doFindBestDriveSpeed"))
        {
            dl.addField("doFindBestDriveSpeed"); dl.newLine();
            dtu.findBestDriveSpeed();
        }

        //Drive motion profile test
        if(testMap.get("doMotionProfile"))
        {
            double d = 24.0;
            double s = 0.7;
            dl.addField("doMotionProfile_"+d+"_"+s); dl.newLine();
            dtu.doMotionProfile(d, s);
        }

        //Find optimal encoder speed
        if(testMap.get("doFindBestEncTurnSpeed"))
        {
            dl.addField("doFindBestEncTurnSpeed"); dl.newLine();
            dtu.findBestEncTurnSpeed();
        }

        //Find optimal gyro speed/gain
        if(testMap.get("doFindBestGyroTurnSpeed"))
        {
            dl.addField("doFindBestGyroTurnSpeed"); dl.newLine();
            dtu.findBestGyroTurnSpeedGain();
        }

        //Test some common distance and turns from auton
        if(testMap.get("doDriveDist"))
        {
            double dist[] = {48.0};
            robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
            for (double aDist : dist)
            {
                drvTrn.setCurrPt(new Point2d(0, 0));
                Point2d tPt = new Point2d(aDist, 0.0);
                dl.addField("doDriveDist " + aDist);
                drvTrn.driveToPointLinear(tPt, 0.55, Drivetrain.Direction.FORWARD);
                drvTrn.driveToTarget(0.2, 10);
            }
        }

        if(testMap.get("doTurnAngle"))
        {
            robot.gyro.resetZAxisIntegrator();
            double angle = 5.0; //-90.0;
            //dl.addField("doTurnAngle ENC " + angle);
            //drvTrn.ctrTurnLinear(angle - robot.getGyroFhdg(), 0.4);
            for(double spd = 0.1; spd <= 0.4; spd+=0.1)
            {
                for (int i = 2; i < 6; i++)
                {
                    int cHdg = robot.getGyroFhdg();
                    double ang = angle * i;
                    dl.addField("doTurnAngle GYRO from " + cHdg + " to " + ang + " at " + spd);
                    dl.newLine();
                    drvTrn.ctrTurnToHeading(ang, spd);
                    sleep(1000);
                    cHdg = robot.getGyroFhdg();
                    dl.addField("doTurnAngle GYRO from " + cHdg + " to " + 0 + " at " + spd);
                    dl.newLine();
                    drvTrn.ctrTurnToHeading(0, spd);
                    sleep(1000);
                }
            }
        }


        if(testMap.get("doMaxSpeedTest"))
        {
            dl.addField("doMaxSpeedTest"); dl.newLine();
            dtu.doMaxSpeedTest(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sleep(2000);
            dtu.doMaxSpeedTest(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(1000);
        }

        if(testMap.get("doMinSpeedTest"))
        {
            dl.addField("doMinSpeedTest"); dl.newLine();
            dtu.doMinSpeedTest();
            sleep(1000);
        }

        if(testMap.get("doSturnTest"))
        {
            dtu.doDoubleCurveTurn(4.0, 0.3);
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
            dl = new DataLogger("driveTest_" + testToRun);
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
        FtcChoiceMenu<String> testMenu = new FtcChoiceMenu<>("TEST:", null, this);

        for(String str : tests)
        {
            testMenu.addChoice(str, str);
        }

        FtcMenu.walkMenuTree(testMenu, this);

        String outStr = testMenu.getCurrentChoiceObject();

        for(String str : tests)
        {
            if(outStr.equals(str))
            {
                testMap.put(str, true);
            }
            else
            {
                testMap.put(str, false);
            }
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
