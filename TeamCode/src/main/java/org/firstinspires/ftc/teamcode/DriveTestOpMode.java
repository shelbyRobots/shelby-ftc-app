package org.firstinspires.ftc.teamcode;

import android.widget.TextView;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.Calendar;
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
        waitForStart();
        startMode();
        stopMode();
    }

    public void startMode()
    {
        dashboard.clearDisplay();
        do_main_loop();
    }

    public void do_main_loop()
    {
        dtu.doMaxSpeedTest(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(5000);
        dtu.doMaxSpeedTest(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(5000);
        dtu.doMinSpeedTest();
        sleep(5000);
        dtu.driveDist(12.0, 0.2, false);
        sleep(5000);
        dtu.driveDist(12.0, 0.2, true);
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
