package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test: Loop Timing", group="3543TestSamples")
public class TimingTest extends LinearOpMode
{
    private enum SensorType
    {
        DRIVEBASE_ENCODERS,
        GYRO
    }

    private static final String TAG = "TrcDbg";
    private static final double DRIVE_POWER = 0.2;
    private static final double TURN_POWER = 0.4;
    private static SensorType sensorType = SensorType.GYRO;
    private static final DcMotor.Direction LEFTWHEEL_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction RIGHTWHEEL_DIRECTION = DcMotor.Direction.REVERSE;

    private DcMotor lrWheel;
    private DcMotor rrWheel;
    private ModernRoboticsI2cGyro gyro;
    //private ColorSensor colorSensor;


    public void runOpMode()
    {
        initRobot();

        waitForStart();

        while (!isStopRequested() &&
               gyro.isCalibrating())
        {
            sleep(50);
        }

        startRobot();


        sleep(1000);  //Give motors time to ramp up to speed

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

        while (opModeIsActive())
        {
            long currTime = System.nanoTime();
            int currSample = getSensorValue();
            long loopInterval = currTime - prevLoopTime;

            if (currSample != prevSample)
            {
                long sampleTime = currTime - prevSampleTime;
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
            }
            else if (loopInterval > maxLoopInterval)
            {
                maxLoopInterval = loopInterval;
            }

            logRobot(String.format("[%4d:%7.3f] LoopInterval=%7.3f, ",
                    loopCount, (currTime - startTime)/1000000.0, loopInterval/1000000.0));

            prevLoopTime = currTime;
            loopCount++;
        }
        stopRobot();

        long endTime = System.nanoTime();
        Log.i(TAG, String.format(
                "Loop: MinInterval=%7.3f, MaxInterval=%7.3f, AvgInterval=%7.3f",
                minLoopInterval/1000000.0, maxLoopInterval/1000000.0,
                (endTime - startTime)/1000000.0/loopCount));
        Log.i(TAG, String.format(
                "Sensor: MinSampleInterval=%7.3f, MaxSampleInterval=%7.3f, AvgSampleInterval=%7.3f %7.3f",
                minSampleInterval/1000000.0, maxSampleInterval/1000000.0,
                (endTime - startTime)/1000000.0/sampleCount,
                (double)totalSampleTime/sampleCount));
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

        lrWheel.setMaxSpeed(3000);
        rrWheel.setMaxSpeed(3000);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.resetZAxisIntegrator();

        //colorSensor = hardwareMap.colorSensor.get("color");
    }

    private int getSensorValue()
    {
        int value = 0;

        switch (sensorType)
        {
            case DRIVEBASE_ENCODERS:
                value = (lrWheel.getCurrentPosition() + rrWheel.getCurrentPosition())/2;
                break;

            case GYRO:
                value = -gyro.getIntegratedZValue();
                break;
        }

        return value;
    }

    private void logRobot(String prefix)
    {
        switch (sensorType)
        {
            case DRIVEBASE_ENCODERS:
                // checking encoders.
                Log.i(TAG, prefix + String.format("lr=%d, rr=%d",
                        lrWheel.getCurrentPosition(), rrWheel.getCurrentPosition()));
                break;

            case GYRO:
                // checking gyro.
                Log.i(TAG, prefix + String.format("heading=%d %d", -gyro.getIntegratedZValue(),
                        gyro.getHeading()));
                break;
        }
    }

    private void startRobot()
    {
       double lspd = 0.0;
       double rspd = 0.0;
        switch (sensorType)
        {
            case DRIVEBASE_ENCODERS:
               // Driving forward
               lspd = DRIVE_POWER;
               rspd = DRIVE_POWER;
               break;
            case GYRO:
               // Turning right
               lspd = TURN_POWER;
               rspd = -TURN_POWER;
               break;
        }
        lrWheel.setPower(lspd);
        rrWheel.setPower(rspd);
    }

    private void stopRobot()
    {
        lrWheel.setPower(0.0);
        rrWheel.setPower(0.0);
    }

}
