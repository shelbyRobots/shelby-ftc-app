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

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Telop Tank", group="Tele")
//@Disabled
public class TeleopTank_Driver extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        double left;                             //driver  left joy
        double right;                            //driver right joy
        double shooter;                          //operator right trigger float
        double bkwr_shooter;

        boolean shoot_pressed;                   //operator right trigger boolean
        boolean flicker_pressed;                 //driver a button
        boolean last_shoot_pressed = false;
        boolean last_flicker_pressed = false;
        boolean flickertoggle = false;
        boolean d_down_last = false;             //operator dpad -> adjust shoot power
        boolean d_up_last = false;
        boolean b_pressed;                       //driver b button -> brake/float toggle
        boolean b_pressed_last = false;
        boolean invert_drive_pressed;            //driver y button -> toggle front/back
        boolean last_invert_drive_pressed = false;
        boolean switch_mode_pressed;             //driver x button -> temporary test for motor mode
        boolean last_switch_mode_pressed = false;
        boolean bkwr_shoot_pressed;              //with prefix bkwr_, variable is used to run shooters backwards
        boolean last_bkwr_shoot_pressed = false;

        double lpushed;
        double rpushed;
        boolean lpush;                           //driver left trigger -> move lpusher left
        boolean rpush;                           //driver left trigger -> move lpusher right
        boolean lpush_last = false;
        boolean rpush_last = false;

        boolean lbump;                           //operator left bumper -> auto shoot
        boolean lbump_last = false;

        Input_Shaper ishaper = new Input_Shaper();

        DcMotor.ZeroPowerBehavior zeroPwr = DcMotor.ZeroPowerBehavior.FLOAT;

        double shoot_scale = 0.75;

        double elev;
        double sweep;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(this);

        if (robot.leftMotor  != null &&
            robot.rightMotor != null &&
            robot.gyro       != null)
        {
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //robot.leftMotor.setMaxSpeed(4000);
            //robot.rightMotor.setMaxSpeed(4000);
            dtrn.init(robot);
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);

        double curLpushPos = L_DN_PUSH_POS;
        double curRpushPos = R_DN_PUSH_POS;
        robot.rpusher.setPosition(curRpushPos);
        robot.lpusher.setPosition(curLpushPos);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean toggle = false;
        boolean bkwtoggle = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
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
            bkwr_shooter = gamepad2.left_trigger;
            lpushed = gamepad1.left_trigger;
            rpushed = gamepad1.right_trigger;
            flicker_pressed = gamepad1.a;

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

            if(shoot_pressed && !last_shoot_pressed)
            {
                toggle = !toggle;

                if(toggle)
                    shooter_motors(shoot_scale);
                else
                    shooter_motors(0.0);
            }
            last_shoot_pressed = shoot_pressed;

            bkwr_shoot_pressed = (bkwr_shooter > 0);

            if(bkwr_shoot_pressed && !last_bkwr_shoot_pressed)
            {
                toggle = !toggle;

                if(toggle)
                    shooter_motors(-1.0);
                else
                    shooter_motors(0.0);
            }
            last_bkwr_shoot_pressed = bkwr_shoot_pressed;

            rpush = (rpushed > 0);

            if(rpush && !rpush_last)
            {
                if(curRpushPos == R_UP_PUSH_POS)
                {
                    curRpushPos = R_DN_PUSH_POS;
                }
                else
                {
                    curRpushPos = R_UP_PUSH_POS;
                }
                robot.rpusher.setPosition(curRpushPos);
            }
            rpush_last = rpush;

            lpush = (lpushed > 0);

            if(lpush && !lpush_last)
            {
                if(curLpushPos == L_UP_PUSH_POS)
                {
                    curLpushPos = L_DN_PUSH_POS;
                }
                else
                {
                    curLpushPos = L_UP_PUSH_POS;
                }
                robot.lpusher.setPosition(curLpushPos);
            }
            lpush_last = lpush;


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
                RobotLog.ii("SJH", "AUTOSHOOT");
                robot.shotmotor1.setPower(shoot_scale);
                robot.shotmotor2.setPower(shoot_scale);
                robot.sweepMotor.setPower(-1.0);
                dtrn.driveDistance(35.0, 0.8, Drivetrain.Direction.REVERSE);
                while(opModeIsActive() && dtrn.isBusy())
                {
                    idle();
                }
                dtrn.stopAndReset();

                RobotLog.ii("SJH", "DONE AUTOSHOOT MOVE");
//                ElapsedTime stimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//                sleep(500);
//                robot.sweepMotor.setPower(-1.0);
//                robot.elevMotor.setPower(-1.0);
//                sleep(1500);
//                robot.shotmotor1.setPower(0);
//                robot.shotmotor2.setPower(0);
//                robot.sweepMotor.setPower(0);
//                robot.elevMotor.setPower(0);
//                RobotLog.ii("SJH", DONE AUTOSHOOT");
            }
            lbump_last = lbump;

            invert_drive_pressed = gamepad1.y;
            if(invert_drive_pressed && !last_invert_drive_pressed)
            {
                robot.invertDriveDir();
            }
            last_invert_drive_pressed = invert_drive_pressed;

            switch_mode_pressed = gamepad1.x;
            if(switch_mode_pressed && !last_switch_mode_pressed)
            {
                DcMotor.RunMode currMode = robot.leftMotor.getMode();
                if(currMode == DcMotor.RunMode.RUN_USING_ENCODER)
                {
                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //robot.leftMotor.setMaxSpeed(4000);
                    //robot.rightMotor.setMaxSpeed(4000);
                }
                else
                {
                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //robot.leftMotor.setMaxSpeed(2650);
                    //robot.rightMotor.setMaxSpeed(2650);
                }
            }
            last_switch_mode_pressed = switch_mode_pressed;

            telemetry.addData("left : ",  "%.2f", left);
            telemetry.addData("right : ", "%.2f", right);
            telemetry.addData("elev : ", elev);
            telemetry.addData("sweep : ",  sweep);
            telemetry.addData("shooters", "%.2f", shooter);
            telemetry.addData("shooters", "%.2f", bkwr_shooter);
            telemetry.addData("shootpwr", "%s", last_bkwr_shoot_pressed);
            telemetry.addData("shootpwr", "%s", last_shoot_pressed);
            telemetry.addData("shoot_scale", "%.2f", shoot_scale);
            telemetry.addData("SHT1CNT", "%5d", robot.shotmotor1.getCurrentPosition());
            telemetry.addData("SHT2CNT", "%5d", robot.shotmotor2.getCurrentPosition());
            telemetry.addData("zmode", "%s", robot.leftMotor.getZeroPowerBehavior());
            telemetry.update();

            // Pause for metronome tick.
            robot.waitForTick(10);
        }
    }


    private void shooter_motors(double speed)
    {
        robot.shotmotor1.setPower(speed);
        robot.shotmotor2.setPower(speed);
    }

    private void bkwr_shooter_motors ()
    {
        robot.shotmotor1.setPower(-1.0);
        robot.shotmotor2.setPower(-1.0);
    }

    private void do_pushButtonright (ButtonSide bside)
    {
        if (bside == ButtonSide.LEFT) {
            robot.rpusher.setPosition(R_UP_PUSH_POS);
        }

        if (bside == ButtonSide.RIGHT) {
            robot.rpusher.setPosition(R_DN_PUSH_POS);
        }
    }

    private void do_pushButtonleft (ButtonSide bside)
    {
        if (bside == ButtonSide.LEFT) {
            robot.lpusher.setPosition(L_UP_PUSH_POS);
        }

        if (bside == ButtonSide.RIGHT) {
            robot.lpusher.setPosition(L_DN_PUSH_POS);
        }
    }

    private enum ButtonSide
    {
        UNKNOWN,
        LEFT,
        RIGHT
    }



    //This line of text has no use. It can be deleted.


    private final static double L_DN_PUSH_POS = 1.0;
    private final static double R_DN_PUSH_POS = 0.05;
    private final static double L_UP_PUSH_POS = 0.05;
    private final static double R_UP_PUSH_POS = 1.0;

    private ShelbyBot robot = new ShelbyBot();
    private Drivetrain dtrn = new Drivetrain();
}
