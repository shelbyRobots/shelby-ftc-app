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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.util.Date;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Drive By Gyro", group="Test")
//@Disabled
public class AutoDriveByGyro_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    ShelbyBot               robot   = new ShelbyBot();
    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 1120;
    static final double     DRIVE_GEAR_REDUCTION    = 0.5;
    static final double     WHEEL_DIAMETER_INCHES   = 4.5;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * Math.PI);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.5;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.4;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
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

    @Override
    public void runOpMode()
    {
        if (logData)
        {
            Date day = new Date();
            dl = new DataLogger(day.toString() + "autonomousData");
            dl.addField("Gyro");
            dl.addField("LENC");
            dl.addField("RENC");
            dl.addField("RED");
            dl.addField("GRN");
            dl.addField("BLU");
            dl.newLine();
        }
        robot.init(hardwareMap);
        gyro = robot.gyro;

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(gyro == null) stop();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

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
        gyro.resetZAxisIntegrator();

        turnColorOff();
        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
        dl.addField("SgyroDrive"); dl.addField("",DD); dl.addField(0.0f); dl.newLine();
        gyroDrive(DRIVE_SPEED, DD, 0.0);    // Drive FWD DD inches
        dl.addField("SgyroTurn"); dl.addField("",-TA); dl.newLine();
        gyroTurn( TURN_SPEED, -TA);         // Turn  CCW to -45 Degrees
        dl.addField("SgyroHold"); dl.addField("",-TA); dl.addField(0.5f); dl.newLine();
        gyroHold( TURN_SPEED, -TA, 0.5);    // Hold -45 Deg heading for a 1/2 second
        dl.addField("SgyroTurn"); dl.addField("",TA); dl.addField(0.0f); dl.newLine();
        gyroTurn( TURN_SPEED,  TA);         // Turn  CW  to  45 Degrees
        dl.addField("SgyroHold"); dl.addField("",TA); dl.addField(0.5f); dl.newLine();
        gyroHold( TURN_SPEED,  TA, 0.5);    // Hold  45 Deg heading for a 1/2 second
        dl.addField("SgyroTurn"); dl.addField("",0.0); ; dl.newLine();
        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        dl.addField("SgyroHold"); dl.addField("",0.0); dl.addField(1.0f); dl.newLine();
        gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        dl.addField("SgyroDrive"); dl.addField("",-DD); dl.addField(0.0f); dl.newLine();
        gyroDrive(DRIVE_SPEED, -DD, 0.0);    // Drive REV 48 inches
        dl.addField("SgyroHold"); dl.addField("",0.0); dl.addField(0.5f); dl.newLine();
        gyroHold( TURN_SPEED,   0.0, 0.5);    // Hold  0 Deg heading for a 1/2 second

        turnColorOn();
        robot.setDriveDir(ShelbyBot.DriveDir.PUSHER);
        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
        dl.addField("SgyroDrive"); dl.addField("",DD); dl.addField(0.0f); dl.newLine();
        gyroDrive(DRIVE_SPEED, DD, 0.0);    // Drive FWD DD inches
        dl.addField("SgyroTurn"); dl.addField("",-TA); dl.newLine();
        gyroTurn( TURN_SPEED, -TA);         // Turn  CCW to -45 Degrees
        dl.addField("SgyroHold"); dl.addField("",-TA); dl.addField(0.5f); dl.newLine();
        gyroHold( TURN_SPEED, -TA, 0.5);    // Hold -45 Deg heading for a 1/2 second
        dl.addField("SgyroTurn"); dl.addField("",TA); dl.addField(0.0f); dl.newLine();
        gyroTurn( TURN_SPEED,  TA);         // Turn  CW  to  45 Degrees
        dl.addField("SgyroHold"); dl.addField("",TA); dl.addField(0.5f); dl.newLine();
        gyroHold( TURN_SPEED,  TA, 0.5);    // Hold  45 Deg heading for a 1/2 second
        dl.addField("SgyroTurn"); dl.addField("",0.0); ; dl.newLine();
        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        dl.addField("SgyroHold"); dl.addField("",0.0); dl.addField(1.0f); dl.newLine();
        gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        dl.addField("SgyroDrive"); dl.addField("",-DD); dl.addField(0.0f); dl.newLine();
        gyroDrive(DRIVE_SPEED, -DD, 0.0);    // Drive REV 48 inches
        dl.addField("SgyroHold"); dl.addField("",0.0); dl.addField(0.5f); dl.newLine();
        gyroHold( TURN_SPEED,   0.0, 0.5);    // Hold  0 Deg heading for a 1/2 second

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
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

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

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

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
                   (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                if(frame%20 == 0)
                {
                    if (colorOn && robot.colorSensor != null)
                    {
                        r = robot.colorSensor.red();
                        g = robot.colorSensor.green();
                        b = robot.colorSensor.blue();
                    }

                    logData();
                }
                frame++;

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

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

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void gyroTurn (  double speed, double angle) {

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            if(frame%20 == 0)
            {
                if (colorOn && robot.colorSensor != null)
                {
                    r = robot.colorSensor.red();
                    g = robot.colorSensor.green();
                    b = robot.colorSensor.blue();
                }

                logData();
            }
            frame++;
            telemetry.update();
        }
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
        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            if(frame%20 == 0)
            {
                if (colorOn && robot.colorSensor != null)
                {
                    r = robot.colorSensor.red();
                    g = robot.colorSensor.green();
                    b = robot.colorSensor.blue();
                }

                logData();
            }
            frame++;
            telemetry.update();
        }

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

        // Send desired speeds to motors.
        robot.leftMotor.setPower(leftSpeed);
        robot.rightMotor.setPower(rightSpeed);

        // Display it for the driver.
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

    private void turnColorOn()
    {
        if(robot.colorSensor == null) return;
        colorOn = true;
        ModernRoboticsI2cColorSensor cs = robot.colorSensor;
        cs.getI2cController().registerForI2cPortReadyCallback(robot.colorSensor,
                robot.getColorPort());

        cs.enableLed(true);
    }

    private void turnColorOff()
    {
        if(robot.colorSensor == null) return;
        colorOn = false;
        ModernRoboticsI2cColorSensor cs = robot.colorSensor;
        cs.enableLed(false);
        cs.getI2cController().deregisterForPortReadyCallback(robot.getColorPort());
        r = 0;
        g = 0;
        b = 0;
    }

    public void logData()
    {
        if (datalogtimer.seconds() <0.01) return;

        datalogtimer.reset();

        if(logData)
        {
            //Write sensor values to the data logger
            if(robot.gyro != null) dl.addField(robot.getGyroFhdg());
            else                   dl.addField("");
            dl.addField(robot.leftMotor.getCurrentPosition());
            dl.addField(robot.rightMotor.getCurrentPosition());
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
