
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        elevMotor   = hwMap.dcMotor.get("elev_drive");
        sweepMotor  = hwMap.dcMotor.get("sweep_drive");
        shotmotor1  = hwMap.dcMotor.get("shot_motor_1");
        shotmotor2  = hwMap.dcMotor.get("shot_motor_2");

        shotmotor2.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);  // FORWARD if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE); // REVERSE if using AndyMark motors
        elevMotor.setDirection(DcMotor.Direction.REVERSE); // REVERSE if using AndyMark motors
        sweepMotor.setDirection(DcMotor.Direction.REVERSE); // REVERSE if using AndyMark motors

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        elevMotor.setPower(0);
        sweepMotor.setPower(0);
        shotmotor1.setPower(0);
        shotmotor2.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sweepMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweepMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        DbgLog.msg("SBH left_drive: \n"
                + "cncInfo " + leftMotor.getConnectionInfo() + "\n"
                + "devName " + leftMotor.getDeviceName() + "\n"
                + "cntrlr  " + leftMotor.getController().toString() + "\n"
                + "maxSpd  " + leftMotor.getMaxSpeed() + "\n"
                + "prtNum  " + leftMotor.getPortNumber() + "\n"
                + "curPos  " + leftMotor.getCurrentPosition() + "\n"
                + "dir     " + leftMotor.getDirection() + "\n");

        DbgLog.msg("SBH right_drive: \n"
                + "cncInfo " + rightMotor.getConnectionInfo() + "\n"
                + "devName " + rightMotor.getDeviceName() + "\n"
                + "cntrlr  " + rightMotor.getController().toString() + "\n"
                + "maxSpd  " + rightMotor.getMaxSpeed() + "\n"
                + "prtNum  " + rightMotor.getPortNumber() + "\n"
                + "curPos  " + rightMotor.getCurrentPosition()
                + "dir     " + rightMotor.getDirection());

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
    private static final float CAMERA_X_IN_BOT = 0.0f  * MM_PER_INCH;
    private static final float CAMERA_Y_IN_BOT = 0.0f; //FRONT_OFFSET * MM_PER_INCH;
    private static final float CAMERA_Z_IN_BOT = 0.0f; //10.6f * MM_PER_INCH;

    //With phone laid flat in portrait mode with screen up:
    //The phone axis is 0,0,0 at Camera (using front camera)
    //X pts to the right side of phone (ZTE volume button edge)
    //Y pts to top of phone (head phone jack edge)
    //Z pts out of camera - initially toward bot up
    //to mount camera on front of bot looking bot fwd,
    //rotate 90 about z, then -90 about x
    //translate 0 in bot x, half bot length in bot y, and ~11" in bot z
    static final OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_X_IN_BOT, CAMERA_Y_IN_BOT, CAMERA_Z_IN_BOT)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.ZXY,
                    AngleUnit.DEGREES, 90, -90, 0));
}
