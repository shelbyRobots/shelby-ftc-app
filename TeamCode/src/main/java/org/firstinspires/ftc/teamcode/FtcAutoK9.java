/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import ftclib.FtcChoiceMenu;
import ftclib.FtcDcMotor;
import ftclib.FtcGamepad;
import ftclib.FtcMRGyro;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;
import hallib.HalDbgLog;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcSensor;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

@Autonomous(name="Auto: K9Bot Various Autonomous", group="Test")
//@Disabled
public class FtcAutoK9 extends FtcOpMode implements TrcPidController.PidInput,
                                                    FtcMenu.MenuButtons
{
    private enum AutoStrategy
    {
        DO_NOTHING,
        TIMED_DRIVE,
        DRIVE_AND_TURN
    }   //enum AutoStrategy

    private enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    private enum TimedDriveState
    {
        DELAY,
        DRIVE,
        DONE
    }   //enum TimedDriveState

    private enum DriveAndTurnState
    {
        DELAY,
        DRIVE,
        TURN,
        DONE
    }   //enum DriveAndTurnState

    //
    // PID drive constants.
    //
    private static final double DRIVE_KP                = 0.35;
    private static final double DRIVE_KI                = 0.0;
    private static final double DRIVE_KD                = 0.0;
    private static final double DRIVE_KF                = 0.0;
    private static final double DRIVE_TOLERANCE         = 1.0;
    private static final double DRIVE_SETTLING          = 0.2;

    private final static double DRV_TUNER = 1.0;
    private final static double WHL_DIAMETER = 6.6 * DRV_TUNER; //Diameter of the wheel (inches)
    private final static int    ENCODER_CPR = ShelbyBot.ENCODER_CPR;
    private final static double GEAR_RATIO  = 1;                   //Gear ratio

    private final static double CIRCUMFERENCE = Math.PI * WHL_DIAMETER;
    private final static double CPI = ENCODER_CPR * GEAR_RATIO / CIRCUMFERENCE;
    private static final double DRIVE_INCHES_PER_COUNT  = 1.0/CPI;

    //
    // PID turn constants.
    //
    private static final double TURN_KP                 = 0.025;
    private static final double TURN_KI                 = 0.0;
    private static final double TURN_KD                 = 0.0;
    private static final double TURN_KF                 = 0.0;
    private static final double TURN_TOLERANCE          = 2.0;
    private static final double TURN_SETTLING           = 0.2;

    private HalDashboard dashboard;

    //
    // Sensors.
    //
    private FtcMRGyro gyro;
    //
    // DriveBase subsystem.
    //
    private FtcDcMotor motorLeft;
    private FtcDcMotor motorRight;
    private TrcDriveBase driveBase;
    //
    // PID drive.
    //
    private TrcPidController drivePidCtrl;
    private TrcPidController turnPidCtrl;
    private TrcPidDrive pidDrive;
    //
    // State machine.
    //
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine sm;
    //
    // Menu choices.
    //
    private double delay = 0.0;
    private AutoStrategy autoStrategy = AutoStrategy.DO_NOTHING;
    private double driveTime = 0.0;
    private double driveDistance = 0.0;
    private double turnDegrees = 0.0;
    private Alliance alliance = Alliance.RED_ALLIANCE;

    private ElapsedTime drvTimer = new ElapsedTime();
    private double dtim = 0.0;
    private double ttim = 0.0;

    private double dpwr = 0.5;

    private void doMenus()
    {
        //
        // Create the menus.
        //
        FtcValueMenu delayMenu = new FtcValueMenu("Delay time:", null, this,
                                                         0.0, 10.0, 1.0, 0.0, "%.0f sec");
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance:", delayMenu, this);
        FtcChoiceMenu strategyMenu = new FtcChoiceMenu("Auto Strategies:", allianceMenu, this);
        FtcValueMenu driveTimeMenu = new FtcValueMenu("Drive time:", strategyMenu, this,
                                                      0.0, 10.0, 1.0, 4.0, "%.0f sec");
        FtcValueMenu pwrMenu = new FtcValueMenu("Pwr:", driveTimeMenu,
                                                       this, 0.05, 0.5, 0.05, 0.5, "%3.1f");
        FtcValueMenu distanceMenu = new FtcValueMenu("Drive distance:", strategyMenu, this,
                                                     1.0, 8.0, 1.0, 1.0, "%.0f ft");
        FtcValueMenu degreesMenu = new FtcValueMenu("Turn degrees", distanceMenu, this,
                                                    -360.0, 360.0, 90.0, 360.0, "%.0f deg");

        delayMenu.setChildMenu(allianceMenu);
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, strategyMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, strategyMenu);
        //strategyMenu.addChoice("Do Nothing", AutoStrategy.DO_NOTHING);
        strategyMenu.addChoice("Timed drive", AutoStrategy.TIMED_DRIVE, driveTimeMenu);
        strategyMenu.addChoice("Drive forward", AutoStrategy.DRIVE_AND_TURN, distanceMenu);
        distanceMenu.setChildMenu(degreesMenu);

        //
        // Walk the menu tree starting with the delay menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(delayMenu);
        //
        // Set choices variables.
        //
        dpwr = pwrMenu.getCurrentValue();
        delay = delayMenu.getCurrentValue();
        autoStrategy = (AutoStrategy)strategyMenu.getCurrentChoiceObject();
        driveTime = driveTimeMenu.getCurrentValue();
        driveDistance = distanceMenu.getCurrentValue() * 12.0;
        turnDegrees = degreesMenu.getCurrentValue();
        alliance = (Alliance)allianceMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(0, "Auto Strategy: %s", strategyMenu.getCurrentChoiceText());
    }   //doMenus

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = getDashboard();
        //
        // Sensors.
        //
        gyro = new FtcMRGyro("gyro");
        gyro.calibrate();
        //
        // DriveBase subsystem.
        //
        motorLeft = new FtcDcMotor("leftdrive");
        motorRight = new FtcDcMotor("rightdrive");
        motorRight.setInverted(true);
        driveBase = new TrcDriveBase(motorLeft, motorRight, gyro);
        driveBase.setYPositionScale(DRIVE_INCHES_PER_COUNT);
        //
        // PID drive.
        //
        drivePidCtrl = new TrcPidController(
                "drivePid",
                DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF,
                DRIVE_TOLERANCE, DRIVE_SETTLING, this);
        turnPidCtrl = new TrcPidController(
                "turnPid",
                TURN_KP, TURN_KI, TURN_KD, TURN_KF,
                TURN_TOLERANCE, TURN_SETTLING, this);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, null, drivePidCtrl, turnPidCtrl);
        //
        // State machine.
        //
        event = new TrcEvent("autoEvent");
        timer = new TrcTimer("autoTimer");
        sm = new TrcStateMachine("autoSM");

        //
        // Choice menus.
        //
        //gamepad = new FtcGamepad("Gamepad", gamepad1, this);
        doMenus();
//        delay = 0.0;
//        dpwr = 0.15;
//        driveTime = 1.4;
//        driveDistance = 3.0*12.0;
//        turnDegrees = 90.0;
//        alliance = Alliance.RED_ALLIANCE;
//        autoStrategy = AutoStrategy.DRIVE_AND_TURN;
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        DbgLog.msg("SJH STARTMODE %f", getOpModeElapsedTime());
        dashboard.clearDisplay();
        gyro.setEnabled(true);
        driveBase.resetPosition();
        //
        // Start the state machine according to the auto strategy.
        //
        switch (autoStrategy)
        {
            case TIMED_DRIVE:
                sm.start(TimedDriveState.DELAY);
                break;

            case DRIVE_AND_TURN:
                sm.start(DriveAndTurnState.DELAY);
                break;
        }
    }   //startMode

    @Override
    public void stopMode()
    {
        DbgLog.msg("SJH STOPMODE %f", getOpModeElapsedTime());
        gyro.setEnabled(false);
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        switch (autoStrategy)
        {
            case TIMED_DRIVE:
                doTimedDrive(delay, driveTime);
                break;

            case DRIVE_AND_TURN:
                doDriveAndTurn(delay, driveDistance, turnDegrees);
                break;
        }
    }   //runContinuous

    //
    // Implements TrcPidController.PidInput
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == drivePidCtrl)
        {
            input = driveBase.getYPosition();
        }
        else if (pidCtrl == turnPidCtrl)
        {
            input = driveBase.getHeading();
        }

        return input;
    }   //getInput

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()
    {
        return gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }   //isMenuBackButton

    //
    // Autonomous strategies.
    //

    private void doTimedDrive(double delay, double driveTime)
    {
        if (sm.isReady())
        {
            TimedDriveState state = (TimedDriveState)sm.getState();
            dashboard.displayPrintf(1, "State: %s", state.toString());
            switch (state)
            {
                case DELAY:
                    DbgLog.msg("SJH TIMEDELAY %f", getOpModeElapsedTime());
                    //
                    // Do delay if necessary.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(TimedDriveState.DRIVE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.addEvent(event);
                        sm.waitForEvents(TimedDriveState.DRIVE);
                    }
                    break;

                case DRIVE:
                    DbgLog.msg("SJH TIMEDRIVE %f", getOpModeElapsedTime());
                    //
                    // Drive the given distance.
                    //
                    driveBase.tankDrive(dpwr, dpwr);
                    timer.set(driveTime, event);
                    sm.addEvent(event);
                    sm.waitForEvents(TimedDriveState.DONE);
                    break;

                case DONE:
                default:
                    DbgLog.msg("SJH TIMEDONE %f", getOpModeElapsedTime());
                    //
                    // We are done.
                    //
                    driveBase.stop();
                    sm.stop();
                    break;
            }
        }
    }   //doTimeDrive

    private void doDriveAndTurn(double delay, double distance, double degrees)
    {
        if (sm.isReady())
        {
            DriveAndTurnState state = (DriveAndTurnState)sm.getState();
            dashboard.displayPrintf(1, "State: %s", state.toString());
            switch (state)
            {
                case DELAY:
                    DbgLog.msg("SJH DTDELAY %f", getOpModeElapsedTime());
                    //
                    // Do delay if necessary.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(DriveAndTurnState.DRIVE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.addEvent(event);
                        sm.waitForEvents(DriveAndTurnState.DRIVE);
                    }
                    drvTimer.reset();
                    break;

                case DRIVE:
                    DbgLog.msg("SJH DTDRIVE %f", getOpModeElapsedTime());
                    //
                    // Drive the given distance.
                    //
                    dtim= drvTimer.seconds();
                    DbgLog.msg("SJH DIN ld %f rd %f %f",
                            motorLeft.getPosition(),
                            motorRight.getPosition(),
                            dtim);
                    drvTimer.reset();
                    drivePidCtrl.printPidInfo();
                    pidDrive.setTarget(distance, 0.0, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(DriveAndTurnState.TURN);
                    break;

                case TURN:
                    DbgLog.msg("SJH DTTURN %f", getOpModeElapsedTime());
                    //
                    // Turn the given degrees.
                    //
                    dtim= drvTimer.seconds();
                    DbgLog.msg("SJH TIN ld %f rd %f %f",
                            motorLeft.getPosition(),
                            motorRight.getPosition(),
                            dtim);
                    DbgLog.msg("SJH GRYO_IN %.2f", gyro.getZHeading().value);
                    drvTimer.reset();
                    pidDrive.setTarget(0.0, degrees, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(DriveAndTurnState.DONE);

                    drvTimer.reset();
                    break;

                case DONE:
                    DbgLog.msg("SJH DTDONE %f", getOpModeElapsedTime());
                default:
                    //
                    // We are done.
                    //
                    dtim= drvTimer.seconds();
                    DbgLog.msg("SJH DONE ld %f rd %f %f",
                            motorLeft.getPosition(),
                            motorRight.getPosition(),
                            dtim);
                    DbgLog.msg("SJH GRYO_OUT %.2f", gyro.getZHeading().value);
                    sm.stop();
                    break;
            }
        }
    }   //doDriveAndTurn
}   //class FtcAutoK9
