/*
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Date;

@Autonomous(name="Auto Drive By Gyro", group="Test")
//@Disabled
public class AutoDriveByGyro_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    ShelbyBot               robot   = new ShelbyBot();

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.5;     // For figuring circumference
    static final double TUNE = 1.05;
    static final double CPI = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION /
                                       (WHEEL_DIAMETER_INCHES * TUNE * Math.PI));

    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.3;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.06;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    private ElapsedTime datalogtimer = new ElapsedTime();
    private DataLogger dl;
    private boolean logData = true;
    private boolean gyroReady = false;
    private boolean colorOn = false;
    private int r, g, b;

    private final double DD = 24.0;
    private final double TA = 45.0;

    static int frame = 0;

    private int curLftTarget = 0;
    private int curRgtTarget = 0;

    ShelbyBot.DriveDir startDir = ShelbyBot.DriveDir.SWEEPER;

    @Override
    public void runOpMode()
    {
        robot.init(this);

        if (logData)
        {
            Date day = new Date();
            dl = new DataLogger(day.toString() + "autonomousData");
            dl.addField("Gyro");
            dl.addField("LENC");
            dl.addField("RENC");
            dl.addField("LPWR");
            dl.addField("RPWR");
            dl.addField("RED");
            dl.addField("GRN");
            dl.addField("BLU");
            dl.newLine();
        }


        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        robot.setDriveDir(startDir);

        boolean gyroReady = robot.calibrateGyro();

        if (robot.colorSensor != null)
        {
            robot.colorSensor.enableLed(false);
            robot.colorSensor.enableLed(true);
        }

        telemetry.addData(">", "Robot Ready.");    //

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!isStarted()) {
            telemetry.addData(":", "Robot Heading = %d", robot.getGyroFhdg());
            telemetry.update();
            idle();
        }
        robot.gyro.resetZAxisIntegrator();

        ShelbyBot.DriveDir ddir = startDir;

        robot.turnColorOff();
        dl.addField(ddir.toString()); dl.newLine();
        gyroDrive(DRIVE_SPEED, DD, 0.0);
        logOverrun(0.25);
        int overLcnt = curLftTarget - robot.leftMotor.getCurrentPosition();
        int overRcnt = curRgtTarget - robot.rightMotor.getCurrentPosition();
        double overLdst = overLcnt/CPI;
        double overRdst = overRcnt/CPI;
        gyroDrive(DRIVE_SPEED, (overLdst + overRdst)/2, 0.0, false);
        sleep(3000);
        gyroTurn( TURN_SPEED, -TA);         // Turn  CCW to -45 Degrees
        logOverrun(1.0); sleep(3000);
        gyroHold( TURN_SPEED, -TA, 0.5);    // Hold -45 Deg heading for a 1/2 second
        logOverrun(1.0); sleep(3000);
        gyroTurn( TURN_SPEED,  TA);         // Turn  CW  to  45 Degrees
        logOverrun(1.0); sleep(3000);
        gyroHold( TURN_SPEED,  TA, 0.5);    // Hold  45 Deg heading for a 1/2 second
        logOverrun(1.0); sleep(3000);
        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        logOverrun(1.0); sleep(3000);
        gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        logOverrun(1.0); sleep(3000);
        gyroDrive(DRIVE_SPEED, -DD, 0.0);    // Drive REV 48 inches
        logOverrun(1.0); sleep(3000);
        gyroHold( TURN_SPEED,   0.0, 0.5);    // Hold  0 Deg heading for a 1/2 second
        logOverrun(1.0); sleep(3000);

        ddir = robot.invertDriveDir();

        robot.turnColorOn();
        dl.addField(ddir.toString()); dl.newLine();
        gyroDrive(DRIVE_SPEED, DD, 0.0);    // Drive FWD DD inches
        logOverrun(1.0); sleep(3000);
        gyroTurn( TURN_SPEED, -TA);         // Turn  CCW to -45 Degrees
        logOverrun(1.0); sleep(3000);
        gyroHold( TURN_SPEED, -TA, 0.5);    // Hold -45 Deg heading for a 1/2 second
        logOverrun(1.0); sleep(3000);
        gyroTurn( TURN_SPEED,  TA);         // Turn  CW  to  45 Degrees
        logOverrun(1.0); sleep(3000);
        gyroHold( TURN_SPEED,  TA, 0.5);    // Hold  45 Deg heading for a 1/2 second
        logOverrun(1.0); sleep(3000);
        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        logOverrun(1.0); sleep(3000);
        gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        logOverrun(1.0); sleep(3000);
        gyroDrive(DRIVE_SPEED, -DD, 0.0);    // Drive REV 48 inches
        logOverrun(1.0); sleep(3000);
        gyroHold( TURN_SPEED,   0.0, 0.5);    // Hold  0 Deg heading for a 1/2 second
        logOverrun(1.0); sleep(3000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive (double speed, double distance, double angle)
    {
        gyroDrive(speed, distance, angle, true);
    }
    public void gyroDrive ( double speed, double distance, double angle, boolean correctHdg)
    {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * CPI);
            newLeftTarget  = robot.leftMotor.getCurrentPosition()  + moveCounts;
            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

            if(logData)
            {
                dl.addField("GyroDrive");
                dl.addField(robot.getDriveDir().toString());
                dl.addField("", distance);
                dl.addField("", angle);
                dl.addField("",moveCounts);
                dl.newLine();
            }

            //robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (robot.leftMotor.isBusy() && robot.rightMotor.isBusy()))
            {
                if(frame%5 == 0)
                {
                    logData();
                }
                frame++;

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                if(!correctHdg)
                {
                    steer = 0;
                }

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotor.setPower(leftSpeed);
                robot.rightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                                                             robot.rightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            curLftTarget = newLeftTarget;
            curRgtTarget = newRightTarget;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            logData();
            dl.addField("DONE");
            dl.newLine();
        }
    }


    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle)
    {
        if(logData)
        {
            dl.addField("GyroTurn");
            dl.addField(robot.getDriveDir().toString());
            dl.addField("", angle);
            dl.newLine();
        }

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF))
        {
            if(frame%5 == 0)
            {
                logData();
            }
            frame++;
            telemetry.update();
        }
        logData();
        dl.addField("DONE");
        dl.newLine();
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dl.addField("GyroHold");
        dl.addField(robot.getDriveDir().toString());
        dl.addField("", angle);
        dl.addField("", holdTime);
        dl.newLine();

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime))
        {
            onHeading(speed, angle, P_TURN_COEFF);
            if(frame%5 == 0)
            {
                logData();
            }
            frame++;
            telemetry.update();
        }
        logData();
        dl.addField("DONE");
        dl.newLine();

        // Stop all motion;
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return reached goal boolean
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        robot.leftMotor.setPower(leftSpeed);
        robot.rightMotor.setPower(rightSpeed);

        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.getGyroFhdg();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return steer scale factor
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void logOverrun(double t)
    {
        dl.addField("LOGGING OVERRUN");
        dl.newLine();
        ElapsedTime et = new ElapsedTime();
        while(et.seconds() < t)
        {
            logData();
            robot.waitForTick(10);
        }
    }

    public void logData()
    {
        double dlTimeout = 0.002;
        if (datalogtimer.seconds() <dlTimeout) return;

        datalogtimer.reset();

        if(logData)
        {
            if (colorOn && robot.colorSensor != null)
            {
                r = robot.colorSensor.red();
                g = robot.colorSensor.green();
                b = robot.colorSensor.blue();
            }

            if(robot.gyro != null) dl.addField(robot.getGyroFhdg());
            else                   dl.addField("");
            dl.addField(robot.leftMotor.getCurrentPosition());
            dl.addField(robot.rightMotor.getCurrentPosition());
            dl.addField("",robot.leftMotor.getPower());
            dl.addField("",robot.rightMotor.getPower());
            if(robot.colorSensor != null)
            {
                dl.addField(r);
                dl.addField(g);
                dl.addField(b);
            }
            dl.newLine();
        }
    }
}
