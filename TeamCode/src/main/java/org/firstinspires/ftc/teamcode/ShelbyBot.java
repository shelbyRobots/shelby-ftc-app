
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Shelbybot
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
class ShelbyBot
{
    private LinearOpMode op = null;
    /* Public OpMode members. */
    DcMotor  leftMotor   = null;
    DcMotor  rightMotor  = null;
    DcMotor  elevMotor   = null;
    DcMotor  sweepMotor  = null;
    DcMotor  shotmotor1  = null;
    DcMotor  shotmotor2  = null;
    Servo    lpusher     = null;
    Servo    rpusher     = null;

    ModernRoboticsI2cGyro        gyro        = null;
    ModernRoboticsI2cColorSensor colorSensor = null;
    DeviceInterfaceModule        dim         = null;

    final static int    ENCODER_CPR = 1120;     //Encoder Counts per Revolution

    final static DcMotor.Direction  LEFT_DIR = DcMotor.Direction.FORWARD;
    final static DcMotor.Direction RIGHT_DIR = DcMotor.Direction.REVERSE;

    boolean gyroReady = false;
    int lastRawGyroHdg = 0;

    /* local OpMode members. */
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    ShelbyBot()
    {
    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode op)
    {
        this.op = op;
        this.hwMap = op.hardwareMap;
        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("leftdrive");
        rightMotor  = hwMap.dcMotor.get("rightdrive");
        elevMotor   = hwMap.dcMotor.get("elevmotor");
        sweepMotor  = hwMap.dcMotor.get("sweepmotor");
        shotmotor1  = hwMap.dcMotor.get("leftshooter");
        shotmotor2  = hwMap.dcMotor.get("rightshooter");
        lpusher     = hwMap.servo.get("lpusher");
        rpusher     = hwMap.servo.get("rpusher");

        gyro        = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
        colorSensor = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("color");

        dim = hwMap.deviceInterfaceModule.get("dim");

        // FORWARD for CCW drive shaft rotation if using AndyMark motors
        // REVERSE for  CW drive shaft rotation if using AndyMark motors
        if(leftMotor  != null)  leftMotor.setDirection(LEFT_DIR);
        if(rightMotor != null) rightMotor.setDirection(RIGHT_DIR);
        if(elevMotor  != null)  elevMotor.setDirection(DcMotor.Direction.REVERSE);
        if(sweepMotor != null) sweepMotor.setDirection(DcMotor.Direction.FORWARD);
        // sweepmotor changed from reverse to forward to run with chain 2/11/17
        if(shotmotor1 != null) shotmotor1.setDirection(DcMotor.Direction.FORWARD);
        if(shotmotor2 != null) shotmotor2.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        if(leftMotor  != null)  leftMotor.setPower(0);
        if(rightMotor != null) rightMotor.setPower(0);
        if(elevMotor  != null)  elevMotor.setPower(0);
        if(sweepMotor != null) sweepMotor.setPower(0);
        if(shotmotor1 != null) shotmotor1.setPower(0);
        if(shotmotor2 != null) shotmotor2.setPower(0);

        if(leftMotor  != null)  leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(rightMotor != null) rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(leftMotor  != null)  leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(rightMotor != null) rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(shotmotor1 != null) shotmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(shotmotor2 != null) shotmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(leftMotor  != null)  leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(rightMotor != null) rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(elevMotor  != null)  elevMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(sweepMotor != null) sweepMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(shotmotor1 != null) shotmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(shotmotor2 != null) shotmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //if(leftMotor  != null)  leftMotor.setMaxSpeed(2650);
        //if(rightMotor != null) rightMotor.setMaxSpeed(2650);

        //SET ACTUAL MAX SPEED AFTER TEST - then set shooter speeds in auton and teleop
        //ALSO SET MAX SPEED for drive in auton
        //if(shotmotor1 != null) shotmotor1.setMaxSpeed(30800);
        //if(shotmotor2 != null) shotmotor2.setMaxSpeed(30800);

        if(colorSensor != null)
        {
            //TODO: SBH - Figure out how to register/deregister if timing shows its needed
            //colorPort = colorSensor.getPort();
            //RobotLog.ii("SJH", "I2C Controller version %d",
            //        colorSensor.getI2cController().getVersion());

            RobotLog.ii("SJH", "COLOR_SENSOR");
            RobotLog.ii("SJH", "ConnectionInfo %s", colorSensor.getConnectionInfo());
            RobotLog.ii("SJH", "I2cAddr %s", Integer.toHexString(colorSensor.getI2cAddress().get8Bit()));
            RobotLog.ii("SJH", "I2cAddr %s", Integer.toHexString(colorSensor.getI2cAddress().get7Bit()));

            colorSensor.enableLed(false);
            colorSensor.enableLed(true);

            turnColorOff();
        }

        if(gyro != null)
        {
            RobotLog.ii("SJH", "GYRO_SENSOR");
            RobotLog.ii("SJH", "ConnectionInfo %s", gyro.getConnectionInfo());
            RobotLog.ii("SJH", "I2cAddr %s", Integer.toHexString(gyro.getI2cAddress().get8Bit()));
            RobotLog.ii("SJH", "I2cAddr %s", Integer.toHexString(gyro.getI2cAddress().get7Bit()));
        }
    }

    void setDriveDir (DriveDir ddir)
    {
        if(ddir == DriveDir.UNKNOWN)
        {
            RobotLog.ee("SJH", "ERROR - setDriveDir UNKNOWN not allowed.  Setting SWEEPER.");
            ddir = DriveDir.SWEEPER;
        }

        if(calibrationDriveDir == DriveDir.UNKNOWN) calibrationDriveDir = ddir;

        if(this.ddir == ddir)
            return;

        this.ddir = ddir;

        RobotLog.ii("SJH", "Setting Drive Direction to " + ddir);

        switch (ddir)
        {
            case PUSHER:
            {
                leftMotor   = hwMap.dcMotor.get("rightdrive");
                rightMotor  = hwMap.dcMotor.get("leftdrive");
                break;
            }
            case SWEEPER:
            case UNKNOWN:
            {
                leftMotor   = hwMap.dcMotor.get("leftdrive");
                rightMotor  = hwMap.dcMotor.get("rightdrive");
            }
        }

        leftMotor.setDirection(LEFT_DIR);
        rightMotor.setDirection(RIGHT_DIR);
    }

    DriveDir invertDriveDir()
    {
        DriveDir inDir  = getDriveDir();
        DriveDir outDir = DriveDir.SWEEPER;

        switch(inDir)
        {
            case SWEEPER: outDir = DriveDir.PUSHER;  break;
            case PUSHER:
            case UNKNOWN:
                outDir = DriveDir.SWEEPER; break;
        }

        RobotLog.ii("SJH", "Changing from %s FWD to %s FWD", inDir, outDir);
        setDriveDir(outDir);
        return outDir;
    }

    void setInitHdg(double initHdg)
    {
        this.initHdg = (int) Math.round(initHdg);
    }

    public boolean calibrateGyro()
    {
        if(gyro == null)
        {
            RobotLog.ee("SJH", "NO GYRO FOUND TO CALIBRATE");
            return false;
        }

        if(calibrationDriveDir == DriveDir.UNKNOWN)
        {
            RobotLog.ii("SJH", "calibrateGyro called without having set a drive Direction. " +
                       "Defaulting to SWEEPER.");
            setDriveDir(DriveDir.SWEEPER);
        }

        RobotLog.ii("SJH", "Starting gyro calibration");
        gyro.calibrate();

        double gyroInitTimout = 5.0;
        boolean gyroCalibTimedout = false;
        ElapsedTime gyroTimer = new ElapsedTime();

        while (!op.isStopRequested() &&
               gyro.isCalibrating())
        {
            op.sleep(50);
            if(gyroTimer.seconds() > gyroInitTimout)
            {
                RobotLog.ii("SJH", "GYRO INIT TIMED OUT!!");
                gyroCalibTimedout = true;
                break;
            }
        }
        RobotLog.ii("SJH", "Gyro calibrated in %4.2f seconds", gyroTimer.seconds());

        boolean gyroReady = !gyroCalibTimedout;
        if(gyroReady) gyro.resetZAxisIntegrator();
        this.gyroReady = gyroReady;
        return gyroReady;
    }

    public int getGyroFhdg()
    {
        if(gyro == null) return 0;
        int dirHdgAdj = 0;
        if(ddir != calibrationDriveDir) dirHdgAdj = 180;

        int rawGyroHdg = gyro.getIntegratedZValue();
        //There have been cases where gyro incorrectly returns 0 for a frame : needs filter
        //Uncomment the following block for a test filter
//        if(rawGyroHdg == 0 &&
//           Math.abs(lastRawGyroHdg) > 30)
//        {
//            rawGyroHdg = lastRawGyroHdg;
//        }
//        else
//        {
//            lastRawGyroHdg = rawGyroHdg;
//        }

        //NOTE:  gyro.getIntegratedZValue is +ve CCW (left turn)
        //WHEN the freaking gyro is mounted upright.
        //Since snowman 2.0 has gyro mounted upside down, we need
        //to negate the value!!
        int cHdg = -rawGyroHdg + initHdg + dirHdgAdj;

        while (cHdg <= -180) cHdg += 360;
        while (cHdg >   180) cHdg -= 360;

        return cHdg;
    }

    enum DriveDir
    {
        UNKNOWN,
        SWEEPER,
        PUSHER
    }

    int getColorPort()
    {
        return colorPort;
    }

    public void turnColorOn()
    {
        RobotLog.ii("SJH", "Turning on colorSensor LED");
        colorEnabled = true;
        //TODO: SBH - Figure out how to register/deregister if timing shows its needed
        //colorSensor.getI2cController().registerForI2cPortReadyCallback(colorSensor,
        //        getColorPort());

        op.sleep(50);
        colorSensor.enableLed(true);
    }

    public void turnColorOff()
    {
        colorEnabled = false;
        colorSensor.enableLed(false);
        op.sleep(50);
        //TODO: SBH - Figure out how to register/deregister if timing shows its needed
        //colorSensor.getI2cController().deregisterForPortReadyCallback(getColorPort());
    }

    DriveDir getDriveDir() { return ddir; }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            op.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }


    //NOTE:  Notes reference center of bot on ground as bot coord frame origin.
    //However, it seems logical to use the center of the rear axis (pivot point)
    private static final float MM_PER_INCH     = 25.4f;
    static final float BOT_WIDTH               = 16.8f; //Vehicle width at rear wheels
    private static final float BOT_LENGTH      = 18.0f;

    //Distance from ctr of rear wheel to tail
    static final float REAR_OFFSET             = 9.0f;
    static final float FRNT_OFFSET             = BOT_LENGTH - REAR_OFFSET;
    private static final float CAMERA_X_IN_BOT = 12.5f  * MM_PER_INCH;
    private static final float CAMERA_Y_IN_BOT = 0f; //12.5f * MM_PER_INCH;
    private static final float CAMERA_Z_IN_BOT = 0f; //15.5f * MM_PER_INCH;

    private int colorPort = 0;
    private DriveDir ddir = DriveDir.UNKNOWN;
    public DriveDir calibrationDriveDir = DriveDir.UNKNOWN;
    private HardwareMap hwMap = null;

    public boolean colorEnabled = false;

    private int initHdg = 0;

    //With phone laid flat in portrait mode with screen up:
    //The phone axis is 0,0,0 at Camera (using front camera)
    //X pts to the right side of phone (ZTE volume button edge)
    //Y pts to top of phone (head phone jack edge)
    //Z pts out of camera - initially toward bot up
    //to mount camera on front of bot looking bot fwd,
    //rotate -90 about z, then -90 about x
    //translate 0 in bot x, half bot length in bot y, and ~11" in bot z
    static final OpenGLMatrix phoneOrientation = Orientation.getRotationMatrix(
            AxesReference.EXTRINSIC, AxesOrder.XYZ, //ZXY
            AngleUnit.DEGREES, 0, 90, 0);

    static final OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_X_IN_BOT, CAMERA_Y_IN_BOT, CAMERA_Z_IN_BOT)
            .multiplied(phoneOrientation);
}
