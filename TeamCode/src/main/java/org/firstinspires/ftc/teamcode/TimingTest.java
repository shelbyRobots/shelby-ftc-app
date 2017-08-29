package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

@TeleOp(name="Test: Loop Timing", group="3543TestSamples")
public class TimingTest extends LinearOpMode
{
    private enum SensorType
    {
        DRIVEBASE_ENCODERS,
        GYRO
    }

    private static final String TAG = "TrcDbg";
    //private static final double DRIVE_POWER = 0.2;
    //private static final double TURN_POWER = 0.4;
    private static final double DEF_POWER = 0.4;
    private static SensorType sensorType = SensorType.GYRO;
    private static final DcMotor.Direction LEFTWHEEL_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction RIGHTWHEEL_DIRECTION = DcMotor.Direction.REVERSE;
    private static final double MS2NS = 1000000.0;

    private DcMotor lrWheel;
    private DcMotor rrWheel;
    private ModernRoboticsI2cGyro gyro;
    private ModernRoboticsI2cColorSensor colorSensor;

    private boolean useSleep = true;
    private int     sleepMs  = 10;

    private boolean doTurn = true;


    public void runOpMode()
    {
        initRobot();

        gyro.calibrate();
        while (!isStopRequested() &&
               gyro.isCalibrating())
        {
            sleep(50);
        }

        waitForStart();

        //startRobot();

        //sleep(1000);  //Give motors time to ramp up to speed

        long minLoopInterval = Long.MAX_VALUE;
        long maxLoopInterval = Long.MIN_VALUE;
        long loopCount = 0;
        long prevLoopTime;

        long minSampleInterval = Long.MAX_VALUE;
        long maxSampleInterval = Long.MIN_VALUE;
        long sampleCount = 0;
        long prevSampleTime;
        long totalSampleTime = 0;

        long startTime = System.nanoTime();
        prevSampleTime = startTime;
        prevLoopTime = startTime;
        int prevSample = getSensorValue();

        ElapsedTime spdTimer = new ElapsedTime();
        double spdTimout = 2.0;
        double curSpd = 0.3;
        long stoppedSleepTime = 0;

        //TODO: SBH - Figure out how to register/deregister if timing shows its needed
        //colorSensor.getI2cController().deregisterForPortReadyCallback(colorSensor.getPort());

        while (opModeIsActive() && curSpd <= 0.35)
        {
            Log.i(TAG, String.format(Locale.US, "TESTING AT SPEED = %5.2f", curSpd));
            startRobot(curSpd);
            while (spdTimer.seconds() < spdTimout)
            {
                long currTime = System.nanoTime();
                int currSample = getSensorValue();
                long loopInterval = currTime - prevLoopTime;
                long sampleTime = 0;
                boolean sampleIsNew = false;

                if (currSample != prevSample)
                {
                    sampleIsNew = true;
                    sampleTime = currTime - prevSampleTime;
                    sampleCount++;
                    prevSample = currSample;
                    totalSampleTime += sampleTime;
                    prevSampleTime = currTime;
                    if (sampleTime < minSampleInterval)
                        minSampleInterval = sampleTime;
                    else if (sampleTime > maxSampleInterval)
                        maxSampleInterval = sampleTime;
                }

                if (loopInterval < minLoopInterval)
                {
                    minLoopInterval = loopInterval;
                } else if (loopInterval > maxLoopInterval)
                {
                    maxLoopInterval = loopInterval;
                }
                if (sampleIsNew)
                {
                    logRobot(String.format(Locale.US, "NEW SAMPLE - sampleTime %7.3f",
                            sampleTime / MS2NS));
                }
                logRobot(String.format(Locale.US, "[%4d:%7.3f] LoopInterval=%7.3f, ",
                        loopCount, (currTime - startTime) / MS2NS, loopInterval / MS2NS));

                prevLoopTime = currTime;
                if (useSleep)
                {
                    long startSleepTime = System.nanoTime();
                    waitForTick(sleepMs);
                    long endSleepTime = System.nanoTime();
                    totalSampleTime -= (endSleepTime - startSleepTime);
                }
                loopCount++;
            }
            stopRobot();
            long startStopSleepTime = System.nanoTime();
            waitForTick(500);
            stoppedSleepTime += (System.nanoTime() - startStopSleepTime);
            spdTimer.reset();
            if(curSpd < 0.15) curSpd += 0.01;
            else curSpd += 0.05;

            //TODO: SBH - Figure out how to register/deregister if timing shows its needed
            //colorSensor.getI2cController()
            //           .registerForI2cPortReadyCallback(colorSensor, colorSensor.getPort());
        }
        stopRobot();

        long endTime = System.nanoTime() - stoppedSleepTime;
        Log.i(TAG, String.format(
                "Loop: MinInterval=%7.3f, MaxInterval=%7.3f, AvgInterval=%7.3f",
                minLoopInterval/MS2NS, maxLoopInterval/MS2NS,
                (endTime - startTime)/MS2NS/loopCount));
        Log.i(TAG, String.format(
                "Sensor: MinSampleInterval=%7.3f, MaxSampleInterval=%7.3f, AvgSampleInterval=%7.3f %7.3f",
                minSampleInterval/MS2NS, maxSampleInterval/MS2NS,
                (endTime - startTime)/MS2NS/sampleCount,
                (double)totalSampleTime/MS2NS/sampleCount));
    }

    private void initRobot()
    {
        lrWheel = hardwareMap.dcMotor.get("leftdrive");
        rrWheel = hardwareMap.dcMotor.get("rightdrive");
        lrWheel.setDirection(LEFTWHEEL_DIRECTION);
        rrWheel.setDirection(RIGHTWHEEL_DIRECTION);

        lrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lrWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lrWheel.setMaxSpeed(3000);
        //rrWheel.setMaxSpeed(3000);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.resetZAxisIntegrator();

        colorSensor = (ModernRoboticsI2cColorSensor)hardwareMap.colorSensor.get("color");
    }

    private int getSensorValue()
    {
        int value = 0;

        switch (sensorType)
        {
            case DRIVEBASE_ENCODERS:
                int lrPos = lrWheel.getCurrentPosition();
                int rrPos = rrWheel.getCurrentPosition();
                value = rrPos;
                if(Math.abs(lrPos) >= Math.abs(rrPos)) value = lrPos;
                //value = (lrWheel.getCurrentPosition() + rrWheel.getCurrentPosition())/2;
                break;

            case GYRO:
                value = -gyro.getIntegratedZValue();
                break;
        }

        return value;
    }

    private void logRobot(String prefix)
    {
        Log.i(TAG, prefix + String.format("heading=%d %d lspd %4.2f rspd %4.2f lPos %d rPos %d",
                -gyro.getIntegratedZValue(),
                gyro.getHeading(),
                lrWheel.getPower(),
                rrWheel.getPower(),
                lrWheel.getCurrentPosition(),
                rrWheel.getCurrentPosition()));
    }

    private void startRobot()
    {
        startRobot(DEF_POWER);
    }

    private void startRobot(double spd)
    {
        double rspd = spd;
        if(doTurn) rspd = -rspd;
        lrWheel.setPower(spd);
        rrWheel.setPower(rspd);
    }

    private void stopRobot()
    {
        lrWheel.setPower(0.0);
        rrWheel.setPower(0.0);
    }

    private ElapsedTime period  = new ElapsedTime();

    void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
