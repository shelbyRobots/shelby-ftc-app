
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
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
 * In this case that robot is a Shelbybot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
class ShelbyBot
{
    /* Public OpMode members. */
    DcMotor  leftMotor   = null;
    DcMotor  rightMotor  = null;
    DcMotor  elevMotor   = null;
    DcMotor  sweepMotor  = null;
    DcMotor  shotmotor1  = null;
    DcMotor  shotmotor2  = null;
    Servo    pusher      = null;
    ModernRoboticsI2cGyro gyro       = null;

    final static int    ENCODER_CPR = 1120;     //Encoder Counts per Revolution

    /* local OpMode members. */
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    ShelbyBot()
    {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap)
    {
        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("leftdrive");
        rightMotor  = hwMap.dcMotor.get("rightdrive");
        elevMotor   = hwMap.dcMotor.get("elevmotor");
        sweepMotor  = hwMap.dcMotor.get("sweepmotor");
        shotmotor1  = hwMap.dcMotor.get("leftshooter");
        shotmotor2  = hwMap.dcMotor.get("rightshooter");
        pusher      = hwMap.servo.get("pusher");
        gyro        = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");

        // FORWARD for CCW drive shaft rotation if using AndyMark motors
        // REVERSE for  CW drive shaft rotation if using AndyMark motors
        if(leftMotor  != null)  leftMotor.setDirection(DcMotor.Direction.FORWARD);
        if(rightMotor != null) rightMotor.setDirection(DcMotor.Direction.REVERSE);
        if(elevMotor  != null)  elevMotor.setDirection(DcMotor.Direction.REVERSE);
        if(sweepMotor != null) sweepMotor.setDirection(DcMotor.Direction.REVERSE);
        if(shotmotor1 != null) shotmotor2.setDirection(DcMotor.Direction.FORWARD);
        if(shotmotor2 != null) shotmotor2.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        if(leftMotor  != null)  leftMotor.setPower(0);
        if(rightMotor != null) rightMotor.setPower(0);
        if(elevMotor  != null)  elevMotor.setPower(0);
        if(sweepMotor != null) sweepMotor.setPower(0);
        if(shotmotor1 != null) shotmotor1.setPower(0);
        if(shotmotor2 != null) shotmotor2.setPower(0);

        if(leftMotor  != null)  leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if(rightMotor != null) rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if(leftMotor  != null)  leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(rightMotor != null) rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(leftMotor  != null)  leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(rightMotor != null) rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(elevMotor  != null)  elevMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(sweepMotor != null) sweepMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(shotmotor1 != null) shotmotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(shotmotor2 != null) shotmotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    void waitForTick(long periodMs) throws InterruptedException
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }


    //NOTE:  Notes reference center of bot on ground as bot coord frame origin.
    //However, it seems logical to use the center of the rear axis (pivot point)
    private static final float MM_PER_INCH     = 25.4f;
    static final float BOT_WIDTH               = 16.5f; //Vehicle width at rear wheels
    private static final float BOT_LENGTH      = 18.0f;

    //Distance from ctr of rear wheel to tail
    static final float REAR_OFFSET             = 3.5f;
    static final float FRNT_OFFSET             = BOT_LENGTH - REAR_OFFSET;
    private static final float CAMERA_X_IN_BOT = 12.5f  * MM_PER_INCH;
    private static final float CAMERA_Y_IN_BOT = 0f; //12.5f * MM_PER_INCH;
    private static final float CAMERA_Z_IN_BOT = 0f; //15.5f * MM_PER_INCH;

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
