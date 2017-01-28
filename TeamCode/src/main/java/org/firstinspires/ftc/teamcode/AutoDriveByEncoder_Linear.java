/*
Copyright (c) 2016 Robert Atkinson

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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Date;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Auto Drive By Encoder", group="Test")
//@Disabled
public class AutoDriveByEncoder_Linear extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private ShelbyBot robot = new ShelbyBot();

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.5;     // For figuring circumference
    static final double CPI = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION /
                                       (WHEEL_DIAMETER_INCHES * Math.PI));

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private ElapsedTime datalogtimer = new ElapsedTime();
    private DataLogger dl;
    private boolean logData = true;
    private boolean gyroReady = false;
    private boolean colorOn = false;
    private int r, g, b;

    private final double DD = 24.0;
    private final double TD = 18 * Math.PI; //360

    static int frame = 0;

    @Override
    public void runOpMode() throws InterruptedException
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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();
        DbgLog.msg("SJH: Strt: %7d %7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());

        if (robot.gyro != null)
        {
            DbgLog.msg("SJH: Starting gyro calibration");
            robot.gyro.calibrate();

            // make sure the gyro is calibrated before continuing
            ElapsedTime gyroTimer = new ElapsedTime();
            double gyroInitTimout = 5.0;
            boolean gyroCalibTimedout = false;
            gyroTimer.reset();
            while (!isStopRequested() &&
                           robot.gyro.isCalibrating())
            {
                sleep(50);
                if (gyroTimer.seconds() > gyroInitTimout)
                {
                    DbgLog.msg("SJH: GYRO INIT TIMED OUT!!");
                    gyroCalibTimedout = true;
                    break;
                }
            }
            DbgLog.msg("SJH: Gyro callibrated in %4.2f seconds", gyroTimer.seconds());

            gyroReady = !gyroCalibTimedout;
            if (gyroReady) robot.gyro.resetZAxisIntegrator();
        }

        if (robot.colorSensor != null)
        {
            robot.colorSensor.enableLed(false);
            robot.colorSensor.enableLed(true);
        }

        while (!isStarted()) {
            telemetry.addData(":", "Robot Heading = %d", robot.getGyroFhdg());
            telemetry.addData("<", "LENC = %5d RENC = %5d", robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();
            logData();
            sleep(50);
        }

        //dl.closeDataLogger();

        waitForStart();
        robot.gyro.resetZAxisIntegrator();

        doTestCycle(ShelbyBot.DriveDir.SWEEPER, DRIVE_SPEED, TURN_SPEED);
        sleep(1000);
        doTestCycle(ShelbyBot.DriveDir.PUSHER, DRIVE_SPEED, TURN_SPEED);
        sleep(1000);
        doTestCycle(ShelbyBot.DriveDir.SWEEPER, DRIVE_SPEED/2, TURN_SPEED/2);
        sleep(1000);
        doTestCycle(ShelbyBot.DriveDir.PUSHER, DRIVE_SPEED/2, TURN_SPEED/2);
        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void doTestCycle(ShelbyBot.DriveDir ddir, double spd, double trnspd)
    {
        robot.setDriveDir(ddir);

        turnColorOn();
        dl.addField(ddir.toString()); dl.addField("FWD"); dl.addField("",DD);
        dl.addField("",spd); dl.addField("",trnspd);dl.newLine();
        telemetry.addData("SEG", "SWEEPER FWD 20");
        encoderDrive(DRIVE_SPEED,  DD,  DD, 30.0);
        turnColorOff();
        dl.addField(ddir.toString()); dl.addField("LFT"); dl.addField("",TD);
        dl.addField("",spd); dl.addField("",trnspd);dl.newLine();
        telemetry.addData("SEG", "SWEEPER LFT 20");
        encoderDrive(TURN_SPEED,  -TD,  TD, 30.0);
        turnColorOn();
        dl.addField(ddir.toString()); dl.addField("BCK"); dl.addField("",DD);
        dl.addField("",spd); dl.addField("",trnspd);dl.newLine();
        telemetry.addData("SEG", "SWEEPER BCK 40");
        encoderDrive(DRIVE_SPEED, -DD, -DD, 30.0);
        //turnColorOff();
        dl.addField(ddir.toString()); dl.addField("RGT"); dl.addField("",TD);
        dl.addField("",spd); dl.addField("",trnspd);dl.newLine();
        telemetry.addData("SEG", "SWEEPER RGT 20");
        encoderDrive(TURN_SPEED,   TD, -TD, 30.0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive())
        {
            newLeftTarget  = robot.leftMotor.getCurrentPosition()  + (int)(leftInches  * CPI);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * CPI);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            DbgLog.msg("SJH: TGT : %7d %7d", newLeftTarget, newRightTarget);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                if(frame%20 == 0)
                {
                    if(colorOn  && robot.colorSensor != null)
                    {
                        r = robot.colorSensor.red();
                        g = robot.colorSensor.green();
                        b = robot.colorSensor.blue();
                    }

                    logData();
                    telemetry.addData("TGT", "TGT %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("POS", "POS %7d :%7d",
                            robot.leftMotor.getCurrentPosition(),
                            robot.rightMotor.getCurrentPosition());
                    if(robot.gyro != null)
                        telemetry.addData("ROT", "%d", robot.getGyroFhdg());
                    if(robot.colorSensor != null)
                        telemetry.addData("RGB", "RGB %3d %3d %3d", r, g, b);
                    DbgLog.msg("SJH: Pos : %7d %7d",
                            robot.leftMotor.getCurrentPosition(),
                            robot.rightMotor.getCurrentPosition());
                    telemetry.update();
                }

                frame++;
            }

            DbgLog.msg("SJH: Done: %7d %7d",
                    robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            dl.addField("DONE"); dl.newLine();
            ElapsedTime et = new ElapsedTime();
            while(et.seconds() < 0.5)
            {
                logData();
                sleep(10);
            }

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);
        }
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
