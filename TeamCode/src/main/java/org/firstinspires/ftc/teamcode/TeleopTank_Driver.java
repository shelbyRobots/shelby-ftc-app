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

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
public class TeleopTank_Driver extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        float left;
        float right;
        double drivetrain_speed;
        double shooter;
        boolean shoot_pressed;
        boolean last_shoot_pressed = false;
        boolean Joystick_state;

        double elev;
        double sweep;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        drvTrn.init(robot.leftMotor, robot.rightMotor);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean toggle = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left  = InSh.Input(-gamepad1.left_stick_y);
            right = InSh.Input(-gamepad1.right_stick_y);
            shooter = gamepad2.right_trigger;

            elev  = gamepad2.left_stick_y;
            sweep = gamepad2.right_stick_y;
            robot.elevMotor.setPower(elev);
            robot.sweepMotor.setPower(sweep);

            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);

            shoot_pressed = (shooter > 0);

            if(shoot_pressed && !last_shoot_pressed)
            {
                toggle = !toggle;

                if(toggle)
                    shooter_motors(1);
                else
                    shooter_motors(0);
            }

            last_shoot_pressed = shoot_pressed;

            telemetry.addData("left : ",  "%.2f", left);
            telemetry.addData("right : ", "%.2f", right);
            telemetry.addData("elev : ", elev);
            telemetry.addData("sweep : ",  sweep);
            telemetry.addData("shooters", "%.2f", shooter);
            telemetry.addData("shootpwr", "%s", last_shoot_pressed);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

        }

    }

    private void drivetrain_speed(float left, float right)
    {
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);
    }

    private void shooter_motors(double speed)
    {
        robot.shotmotor1.setPower(speed);
        robot.shotmotor2.setPower(speed);
    }
    private ShelbyBot robot = new ShelbyBot();
    private Drivetrain drvTrn = new Drivetrain();
    private Input_Shapper InSh = new Input_Shapper();
}