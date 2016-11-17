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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Telop Tank", group="Tele")
//@Disabled
public class TeleopTank_Driver extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        double left;
        double right;
        double shooter;
        double bkwshooter;
        boolean shoot_pressed;
        boolean bkwshoot_pressed;
        boolean last_shoot_pressed = false;
        boolean last_bkwshoot_pressed = false;
        boolean d_down_last = false;
        boolean d_up_last = false;
        boolean b_pressed;
        boolean b_pressed_last = false;

        boolean lpush;
        boolean rpush;
        boolean lpush_last = false;
        boolean rpush_last = false;

        boolean lbump;
        boolean lbump_last = false;

        Input_Shaper ishaper = new Input_Shaper();

        DcMotor.ZeroPowerBehavior zeroPwr = DcMotor.ZeroPowerBehavior.FLOAT;

        double shoot_scale = 0.55;

        double elev;
        double sweep;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        if (robot.leftMotor  != null &&
            robot.rightMotor != null &&
            robot.gyro       != null)
        {
            dtrn.init(robot.leftMotor, robot.rightMotor, robot.gyro);
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean toggle = false;
        boolean bkwtoggle = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode
            // (note: The joystick goes negative when pushed forwards, so negate it)
            if(gamepad2.dpad_down && !d_down_last)   shoot_scale -= 0.05;
            else if(gamepad2.dpad_up && !d_up_last)  shoot_scale += 0.05;

            shoot_scale = Range.clip(shoot_scale, 0.0, 1.0);
            d_down_last = gamepad2.dpad_down;
            d_up_last   = gamepad2.dpad_up;

            left  = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            shooter = gamepad2.right_trigger;
            bkwshooter = gamepad2.left_trigger;

            b_pressed = gamepad1.b;

            left  = ishaper.shape(left);
            right = ishaper.shape(right);
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            elev  = gamepad2.left_stick_y;
            sweep = gamepad2.right_stick_y;
            robot.elevMotor.setPower(elev);
            robot.sweepMotor.setPower(sweep);

            shoot_pressed = (shooter > 0);
            bkwshoot_pressed = (bkwshooter > 0);

            if(shoot_pressed && !last_shoot_pressed)
            {
                toggle = !toggle;

                if(toggle)
                    shooter_motors(shoot_scale);
                else
                    shooter_motors(0.0);
            }
            last_shoot_pressed = shoot_pressed;

            if(bkwshoot_pressed && !last_bkwshoot_pressed)
            {
                bkwtoggle = !bkwtoggle;

                if(bkwtoggle)
                    shooter_motors(-0.7);
                else
                    shooter_motors(0.0);
            }
            last_bkwshoot_pressed = shoot_pressed;

            lpush = gamepad1.left_trigger  > 0.1;
            rpush = gamepad1.right_trigger > 0.1;
            if(lpush && !lpush_last)
            {
                do_pushButton(ButtonSide.LEFT);
            }
            else if (rpush && !rpush_last)
            {
                do_pushButton(ButtonSide.RIGHT);
            }
            lpush_last = lpush;
            rpush_last = rpush;

            if(b_pressed && !b_pressed_last)
            {
                if(zeroPwr == DcMotor.ZeroPowerBehavior.BRAKE)
                    zeroPwr = DcMotor.ZeroPowerBehavior.FLOAT;
                else
                    zeroPwr = DcMotor.ZeroPowerBehavior.BRAKE;
                robot.leftMotor.setZeroPowerBehavior(zeroPwr);
                robot.rightMotor.setZeroPowerBehavior(zeroPwr);
            }
            b_pressed_last = b_pressed;

            lbump = gamepad2.left_bumper;
            if(lbump && !lbump_last)
            {
                dtrn.driveDistance(48.0, 0.5, Drivetrain.Direction.REVERSE);
                while(opModeIsActive() && dtrn.isBusy())
                {
                    idle();
                }
                ElapsedTime stimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                robot.shotmotor1.setPower(shoot_scale);
                robot.shotmotor2.setPower(shoot_scale);
                sleep(500);
                robot.sweepMotor.setPower(-1.0);
                robot.elevMotor.setPower(-1.0);
                sleep(1500);
                robot.shotmotor1.setPower(0);
                robot.shotmotor2.setPower(0);
                robot.sweepMotor.setPower(0);
                robot.elevMotor.setPower(0);
            }
            lbump_last = lbump;

            telemetry.addData("left : ",  "%.2f", left);
            telemetry.addData("right : ", "%.2f", right);
            telemetry.addData("elev : ", elev);
            telemetry.addData("sweep : ",  sweep);
            telemetry.addData("shooters", "%.2f", shooter);
            telemetry.addData("shootpwr", "%s", last_shoot_pressed);
            telemetry.addData("shoot_scale", "%.2f", shoot_scale);
            telemetry.addData("zmode", "%s", robot.leftMotor.getZeroPowerBehavior());
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    private void shooter_motors(double speed)
    {
        robot.shotmotor1.setPower(speed);
        robot.shotmotor2.setPower(speed);
    }

    private void do_pushButton(ButtonSide bside)
    {
        DbgLog.msg("SJH: PUSH BUTTON!!!");
        if (bside == ButtonSide.LEFT)
        {
            robot.pusher.setPosition(LEFT_POS);
            DbgLog.msg("SJH: Pushing left button");
        }
        else if (bside == ButtonSide.RIGHT)
        {
            robot.pusher.setPosition(RIGHT_POS);
            DbgLog.msg("SJH: Pushing right button");
        }
    }

    //This line of text has no use. It can be deleted.

    private enum ButtonSide
    {
        UNKNOWN,
        LEFT,
        RIGHT
    }

    private static final double LEFT_POS        = 0.8;
    private static final double RIGHT_POS       = 0.2;

    private ShelbyBot robot = new ShelbyBot();
    private Drivetrain dtrn = new Drivetrain();
}