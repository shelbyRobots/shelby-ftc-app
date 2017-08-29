package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import java.util.Date;

/**
 * Core OpMode class containing most OpenCV functionality
 */
@SuppressWarnings("WeakerAccess")
@Autonomous(name="OpenCVAuton", group ="Test")
public class OpenCVAuton extends OpenCvCameraOpMode
{
    private final static double L_DN_PUSH_POS = 1.0;
    private final static double R_DN_PUSH_POS = 0.0;
    private final static double L_UP_PUSH_POS = 0.0;
    private final static double R_UP_PUSH_POS = 1.0;

    private BeaconFinder bd;
    private ShelbyBot   robot = new ShelbyBot();
    private Drivetrain drvTrn = new Drivetrain();

    private boolean useMotor  = false;
    private boolean gyroReady = false;
    private boolean follow    = false;

    private DataLogger dl;
    private boolean logData = true;

    private void setupLogger()
    {
        if (logData)
        {
            Date day = new Date();
            dl = new DataLogger(day.toString() + "autonomousData");
            dl.addField("NOTE");
            dl.addField("FRAME");
            dl.addField("Gyro");
            dl.addField("LENC");
            dl.addField("RENC");
            dl.addField("LPWR");
            dl.addField("RPWR");
            dl.addField("RED");
            dl.addField("GRN");
            dl.addField("BLU");
            dl.addField("ESTX");
            dl.addField("ESTY");
            dl.addField("ESTH");
            dl.newLine();
        }
    }

    public void runOpMode()
    {
        initOpenCv();

        setupLogger();
        drvTrn.setDataLogger(dl);
        drvTrn.setOpMode( this );

        flipImage = false;

        imgProc = new BeaconDetector();
        bd = (BeaconFinder) imgProc;

        if ( useMotor ) {
            robot.init(this);
            drvTrn.init(robot);
            gyroReady = robot.calibrateGyro();
        }

        imgProc.setTelemetry(telemetry);

        waitForStart();

        BeaconFinder.BeaconSide blueSide = BeaconFinder.BeaconSide.UNKNOWN;
        BeaconFinder.BeaconSide redSide  = BeaconFinder.BeaconSide.UNKNOWN;
        BeaconFinder.BeaconSide pushSide = BeaconFinder.BeaconSide.UNKNOWN;

        double baseSpeed = 0.2;

        double bConf, zPos = 0, zOff, xPos, xOff, nPos = 0, rDv = 0, lDv = 0;
        double tPow = 0, nAng = 0, dDist = 0;
        double curDistCount = 0.0;
        double cHdg, hErr = 0.0, nOff;

        String beaconStep = "WAIT";

        if ( useMotor ) {
            robot.setDriveDir(ShelbyBot.DriveDir.PUSHER);
            robot.gyro.resetZAxisIntegrator();
        }

        imgProc.startSensing();

        sleep( 200 );

        while(opModeIsActive())
        {
            imgProc.logDebug();
            imgProc.logTelemetry();

            if (useMotor)
            {
                if ( follow ) {
                    if ( bd.getBeaconConf() > 0.25 && bd.getBeaconPosZ() > 12.0 ) {
                        zOff = 1.0 - bd.getBeaconPosZ() / 24.0;
                        xOff = bd.getBeaconPosX();
                        robot.rightMotor.setPower( baseSpeed + zOff * xOff * 0.0025 );
                        robot.leftMotor.setPower( baseSpeed - zOff * xOff * 0.0025 );
                    } else {
                        robot.rightMotor.setPower( 0.0 );
                        robot.leftMotor.setPower( 0.0 );
                    }
                }
                else
                {
                    if (gamepad1.b)
                        beaconStep = "INIT";

                    if (gamepad1.y)
                        robot.gyro.resetZAxisIntegrator();

                    switch (beaconStep) {
                        case "INIT":

                            blueSide = BeaconFinder.BeaconSide.UNKNOWN;
                            redSide  = BeaconFinder.BeaconSide.UNKNOWN;
                            pushSide = BeaconFinder.BeaconSide.UNKNOWN;

                            robot.rpusher.setPosition(R_DN_PUSH_POS);
                            robot.lpusher.setPosition(L_DN_PUSH_POS);

                            curDistCount = 0.0;
                            drvTrn.stopAndReset();

                            cHdg = robot.getGyroFhdg() % 90;
                            hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                            xPos = bd.getBeaconPosX();
                            zPos = bd.getBeaconPosZ();

                            nOff = zPos * Math.tan( Math.toRadians( hErr ) );
                            nPos = xPos + nOff;
                            nAng = Math.atan( nPos / 9.0 );
                            dDist = ( zPos / 4.0 ) / Math.cos( nAng );
                            tPow = 2.0 * nAng / Math.PI;

                            rDv = Range.clip( baseSpeed + tPow, -0.35, 0.35 );
                            lDv = Range.clip( baseSpeed - tPow, -0.35, 0.35 );

                            RobotLog.ii("SJH", "/BEACON/INIT > nOff: %5.2f, nPos: %5.2f, nAng: %5.2f, dDist: %5.2f", nOff, nPos, nAng, dDist );

                            robot.rightMotor.setPower( rDv );
                            robot.leftMotor.setPower( lDv );

                            beaconStep = "CENTER";
                            break;

                        case "CENTER":

                            cHdg = robot.getGyroFhdg() % 90;
                            hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                            curDistCount = (robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition()) / 2.0;

                            RobotLog.ii("SJH", "/BEACON/CENTER > r: %5.2f, l: %5.2f, d: %5.2f, a: %5.2f", rDv, lDv, drvTrn.countsToDistance(curDistCount), hErr );

                            if (drvTrn.countsToDistance(curDistCount) > zPos / 2.0) {

                                drvTrn.ctrTurnToHeading( 180, 0.15 );

                                rDv = baseSpeed;
                                lDv = baseSpeed;

                                robot.rightMotor.setPower( rDv );
                                robot.leftMotor.setPower( lDv );

                                beaconStep = "DRIVE";
                            }

                            break;

                        case "ALIGN":

                            cHdg = robot.getGyroFhdg() % 90;
                            hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                            curDistCount = (robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition()) / 2.0;

                            RobotLog.ii("SJH", "/BEACON/ALIGN > r: %5.2f, l: %5.2f, d: %5.2f, a: %5.2f", rDv, lDv, drvTrn.countsToDistance(curDistCount), hErr );

                            if ( Math.abs( hErr ) < 2.0 ) {

                                rDv = baseSpeed;
                                lDv = baseSpeed;

                                robot.rightMotor.setPower( rDv );
                                robot.leftMotor.setPower( lDv );

                                beaconStep = "DRIVE";
                            }

                            break;

                        case "DRIVE":

                            cHdg = robot.getGyroFhdg() % 90;
                            hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                            curDistCount = (robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition()) / 2.0;

                            RobotLog.ii("SJH", "/BEACON/DRIVE > r: %5.2f, l: %5.2f, d: %5.2f, a: %5.2f", rDv, lDv, drvTrn.countsToDistance(curDistCount), hErr );

                            robot.rightMotor.setPower( baseSpeed );
                            robot.leftMotor.setPower( baseSpeed );

                            if (drvTrn.countsToDistance(curDistCount) > zPos) {
                                beaconStep = "WAIT";
                                robot.rightMotor.setPower(0);
                                robot.leftMotor.setPower(0);
                            }

                            break;

                    }

                    if ( beaconStep != "WAIT ") {

                        blueSide = bd.getBluePosSide();
                        redSide = bd.getRedPosSide();

                        // Good when the beacon is in view enough or at least
                        // some driving done.
                        if (pushSide == BeaconFinder.BeaconSide.UNKNOWN &&
                                blueSide != BeaconFinder.BeaconSide.UNKNOWN &&
                                redSide != BeaconFinder.BeaconSide.UNKNOWN &&
                                (beaconStep.equals("ALIGN") || beaconStep.equals("CENTER"))) {

                            pushSide = redSide;

                            switch (pushSide) {

                                case LEFT:
                                    robot.lpusher.setPosition(L_UP_PUSH_POS);
                                    robot.rpusher.setPosition(R_DN_PUSH_POS);
                                    break;

                                case RIGHT:
                                    robot.rpusher.setPosition(R_UP_PUSH_POS);
                                    robot.lpusher.setPosition(L_DN_PUSH_POS);
                                    break;
                            }

                            RobotLog.ii("SJH", "/BEACON/FORWARD > FOUND ON %s", pushSide);

                        }

                        if (drvTrn.countsToDistance(curDistCount) > 5.0 && drvTrn.areDriveMotorsStuck() && drvTrn.isBusy()) {
                            beaconStep = "WAIT";
                            robot.rightMotor.setPower(0);
                            robot.leftMotor.setPower(0);
                            RobotLog.ii("SJH", "/BEACON/FORWARD > NOT MOVING? YIKES! %5.2f", curDistCount);
                        }
                    }
                }
            }

            sleep( 10 );
        }

        imgProc.stopSensing();
        cleanupCamera();
    }
}
