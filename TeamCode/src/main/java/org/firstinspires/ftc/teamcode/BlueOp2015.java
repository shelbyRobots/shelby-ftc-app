package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorController;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Concept: BlueOp2015", group = "Concept")
//@Disabled
public class BlueOp2015 extends OpMode{

    private DcMotor left_drive;
    private DcMotor right_drive;
    //private Servo bucket_servo;
    //private Servo right_servo;
   // private DcMotorController dc_drive_controller;

    final static int ENCODER_CPR = 1440;    //Encoder Counts per Revolution
    final static double GEAR_RATIO = 1;        //Gear ratio
    final static double WHEEL_DIAMETER = 6.5;    //Diameter of the wheel in inches
    final static int DISTANCE = 18;            //Distance in inches to drive

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    double ROTATIONS = DISTANCE / CIRCUMFERENCE;
    double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
    double currentTime;

    ElapsedTime time;

    enum State {driving18, firstturn45, driving55, secturn45, drive24, reverse24, turn90rev, backup, dump, down, park1, park2, park3, park4, done};
    State state;

    @Override
    public void init() {
        //dc_drive_controller = hardwareMap.dcMotorController.get("drive_controller");
        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        //bucket_servo = hardwareMap.servo.get("bucket_servo");
        //right_servo = hardwareMap.servo.get("right_servo");
        left_drive.setDirection(DcMotor.Direction.REVERSE);

        time = new ElapsedTime();
        state = State.driving18;

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //right_servo.setPosition(1);
        //bucket_servo.setPosition(1);
    }

    @Override
    public void loop() {
        currentTime = time.time();

        switch(state) {
            case driving18:
                left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //COUNTS = ((18 / CIRCUMFERENCE) * 1440);
                left_drive.setTargetPosition((int) COUNTS);
                right_drive.setTargetPosition((int) COUNTS);

                //setting motor to reverse doesn't work, need to use negative value for right motor
                left_drive.setPower(0.5);
                right_drive.setPower(0.5);

                //need to give enough time for this state to run before switching to next state
                if (currentTime > 1.5) {
                    state = State.firstturn45;
                    time.reset();
                }
                break;

            case firstturn45:
                left_drive.setPower(0);
                right_drive.setPower(0);
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //have to put in pause to ensure encoder reset works ... could move this to align with time reset as well
                if (currentTime > 0.25) {
                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    COUNTS = 1100;
                    left_drive.setTargetPosition((int) COUNTS);
                    left_drive.setPower(0.5);
                    right_drive.setPower(0);

                    if (currentTime > 1.25) {
                        state = State.driving55;
                        time.reset();
                    }
                }
                break;

            case driving55:
                left_drive.setPower(0);
                right_drive.setPower(0);
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if (currentTime > 0.25) {
                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //in this case, going forward 55 inches vs. using exact encoder counts
                    COUNTS = ((55 / CIRCUMFERENCE) * 1440);
                    //need to set target position in each case
                    left_drive.setTargetPosition((int) COUNTS);
                    right_drive.setTargetPosition((int) COUNTS);
                    //look at the 18" run, the motor powers are opposite this one, don't know why, but it works
                    left_drive.setPower(0.45);
                    right_drive.setPower(0.45);

                    if (currentTime > 2.75) {
                        state = State.secturn45;
                        time.reset();
                    }
                }
                break;

            case secturn45:
                left_drive.setPower(0);
                right_drive.setPower(0);
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if (currentTime > 0.25) {
                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    COUNTS = 2200;
                    right_drive.setTargetPosition((int) COUNTS);
                    left_drive.setTargetPosition((int) (COUNTS / 2));
                    right_drive.setPower(0.5);
                    left_drive.setPower(0.2);

                    if (currentTime > 2) {
                        state = State.drive24;
                        time.reset();
                    }
                }
                break;

            case drive24:
                left_drive.setPower(0);
                right_drive.setPower(0);
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(currentTime > 0.25) {
                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    COUNTS = ((int) ((14 / CIRCUMFERENCE) * 1440));
                    left_drive.setTargetPosition((int) COUNTS);
                    right_drive.setTargetPosition((int) COUNTS);
                    left_drive.setPower(0.4);
                    right_drive.setPower(0.4);

                    if (currentTime > 1.75) {
                        state = State.reverse24;
                        time.reset();
                    }
                }
                break;

            case reverse24:
                left_drive.setPower(0);
                right_drive.setPower(0);
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(currentTime > 0.25) {
                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //need to use a negative encoder count to go backwards
                    COUNTS = ((int) ((9 / CIRCUMFERENCE) * -1440));
                    left_drive.setTargetPosition((int) COUNTS);
                    right_drive.setTargetPosition((int) COUNTS);
                    left_drive.setPower(-0.5);
                    right_drive.setPower(-0.5);

                    if (currentTime > 1.25) {
                        state = State.turn90rev;
                        time.reset();
                    }
                }
                break;

            case turn90rev:
                left_drive.setPower(0);
                right_drive.setPower(0);
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(currentTime > 0.25) {
                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    COUNTS = -2200;
                    left_drive.setTargetPosition((int) COUNTS);
                    left_drive.setPower(-0.5);
                    right_drive.setPower(0);

                    if (currentTime > 2) {
                        state = State.backup;
                        time.reset();
                    }
                }
                break;

            case backup:
                left_drive.setPower(0);
                right_drive.setPower(0);
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(currentTime > 0.25) {
                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //COUNTS = ((4 / CIRCUMFERENCE) * 1440);
                    COUNTS = -240;
                    left_drive.setTargetPosition((int) COUNTS);
                    right_drive.setTargetPosition((int) COUNTS);
                    left_drive.setPower(-0.5);
                    right_drive.setPower(-0.5);

                    if (currentTime > 1) {
                        state = State.dump;
                        time.reset();
                    }
                }
                break;

            case dump:
                left_drive.setPower(0);
                right_drive.setPower(0);

                //bucket_servo.setPosition(0.15);

                //wait then bring the bucket to normal position
                if (currentTime > 1) {
                    state = State.down;
                    time.reset();
                }
                break;

            case down:
                //bucket_servo.setPosition(1);

                if (currentTime > 1) {
                    state = State.done;
                    time.reset();
                }
                break;

            case park1:
                left_drive.setPower(0);
                right_drive.setPower(0);
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(currentTime > 0.25) {
                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //move out of the way of the partner bot, park in the debris zone near ramp
                    COUNTS = 2200;
                    right_drive.setTargetPosition((int) COUNTS);
                    left_drive.setTargetPosition((int) COUNTS);
                    left_drive.setPower(0.5);
                    right_drive.setPower(0);

                    if (currentTime > 2.25) {
                        state = State.park2;
                        time.reset();
                    }
                }
                break;

            case park2:
                left_drive.setPower(0);
                right_drive.setPower(0);
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if (currentTime > 0.25) {
                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    COUNTS = ((int) ((22 / CIRCUMFERENCE) * -1440));
                    left_drive.setTargetPosition((int) COUNTS);
                    right_drive.setTargetPosition((int) COUNTS);
                    right_drive.setPower(-0.5);
                    left_drive.setPower(-0.5);

                    if (currentTime > 2) {
                        state = State.park3;
                        time.reset();
                    }
                }
                break;

            case park3:
                left_drive.setPower(0);
                right_drive.setPower(0);
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if (currentTime > 0.25) {
                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    COUNTS = -2200;
                    left_drive.setTargetPosition((int) COUNTS);
                    right_drive.setTargetPosition((int) COUNTS);
                    left_drive.setPower(-0.5);
                    right_drive.setPower(0);

                    if (currentTime > 2) {
                        state = State.park4;
                        time.reset();
                    }
                }
                break;

            case park4:
                left_drive.setPower(0);
                right_drive.setPower(0);
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if (currentTime > 0.25) {
                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    COUNTS = -360;
                    left_drive.setTargetPosition((int) COUNTS);
                    right_drive.setTargetPosition((int) COUNTS);
                    left_drive.setPower(-0.5);
                    right_drive.setPower(-0.5);

                    if (currentTime > 2) {
                        state = State.done;
                        time.reset();
                    }
                }
                break;

            //have to have a DONE state
            case done:
                left_drive.setPower(0);
                right_drive.setPower(0);
                break;

        }
        telemetry.addData("RunTime", getRuntime());
        telemetry.addData("CurrentTime", currentTime);
    }
}