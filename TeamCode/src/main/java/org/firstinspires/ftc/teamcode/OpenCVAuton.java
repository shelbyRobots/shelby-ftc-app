package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

/**
 * Core OpMode class containing most OpenCV functionality
 */
@SuppressWarnings("WeakerAccess")
@Autonomous(name="OpenCVAuton", group ="Test")
public class OpenCVAuton extends OpenCvCameraOpMode
{
    private BeaconFinder bd;
    private ShelbyBot   robot = new ShelbyBot();
    private Drivetrain drvTrn = new Drivetrain();

    private boolean useMotor  = false;
    private boolean gyroReady = false;
    private boolean follow    = true;

    public void runOpMode()
    {
        initOpenCv();

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

        double baseSpeed = 0.4;

        double bConf, zPos, zOff, xPos, xOff, nPos = 0, rDv = 0, lDv = 0;
        double tPow1 = 0, tPow2 = 0, nAng = 0, dDist = 0;
        double curDistCount = 0.0;
        double cHdg, hErr, nOff;

        String beaconStep = "WAIT";

        if ( useMotor )
            robot.gyro.resetZAxisIntegrator();

        imgProc.startSensing();

        sleep( 200 );

        while(opModeIsActive())
        {
            imgProc.logDebug();
            imgProc.logTelemetry();

            telemetry.addData( "RDV", "%5.2f", rDv );
            telemetry.addData( "LDV", "%5.2f", lDv );
            telemetry.addData( "DIST", "%5.2f", (double) drvTrn.countsToDistance(curDistCount) );
            telemetry.update();

            if (useMotor)
            {
                if ( follow ) {
                    if ( bd.getBeaconConf() > 0.25 && bd.getBeaconPosZ() < 0.7 ) {
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

                            robot.lpusher.setPosition(0.1);

                            curDistCount = 0.0;
                            drvTrn.stopAndReset();

                            cHdg = robot.getGyroFhdg() % 90;
                            hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                            xPos = bd.getBeaconPosX();
                            zPos = bd.getBeaconPosZ();

                            nOff = zPos * Math.tan( Math.toRadians( hErr ) );
                            nPos = xPos / 17.7; // + nOff;
                            nAng = Math.atan( nPos / 9.0 );
                            dDist = (zPos / 4.0) / Math.cos( nAng );
                            tPow1 = 2.0 * nAng / Math.PI; //2 * ( nAng - Math.toRadians( hErr ) ) / Math.PI;
                            tPow2 = 2.0 * nAng / Math.PI;

                            rDv = Range.clip( baseSpeed + tPow1, -1.0, 1.0 );
                            lDv = Range.clip( baseSpeed - tPow1, -1.0, 1.0 );

                            DbgLog.msg("SJH: /BEACON/INIT > nOff: %5.2f, nPos: %5.2f, nAng: %5.2f, dDist: %5.2f", nOff, nPos, nAng, dDist );

                            robot.rightMotor.setPower(rDv);
                            robot.leftMotor.setPower(lDv);

                            beaconStep = "CENTER";
                            break;

                        case "CENTER":

                            cHdg = robot.getGyroFhdg() % 90;
                            hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                            curDistCount = (robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition()) / 2.0;

                            DbgLog.msg("SJH: /BEACON/CENTER > r: %5.2f, l: %5.2f, d: %5.2f, a: %5.2f", rDv, lDv, drvTrn.countsToDistance(curDistCount), hErr );

                            if (drvTrn.countsToDistance(curDistCount) > dDist) {

                                rDv = Range.clip( baseSpeed - tPow2, -1.0, 1.0 );
                                lDv = Range.clip( baseSpeed + tPow2, -1.0, 1.0 );

                                robot.rightMotor.setPower(rDv);
                                robot.leftMotor.setPower(lDv);

                                beaconStep = "ALIGN";
                            }

                            break;

                        case "ALIGN":

                            cHdg = robot.getGyroFhdg() % 90;
                            hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                            curDistCount = (robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition()) / 2.0;

                            DbgLog.msg("SJH: /BEACON/ALIGN > r: %5.2f, l: %5.2f, d: %5.2f, a: %5.2f", rDv, lDv, drvTrn.countsToDistance(curDistCount), hErr );

                            if (drvTrn.countsToDistance(curDistCount) > dDist * 1.8) {

                                beaconStep = "DRIVE";
                                robot.rightMotor.setPower(0.25);
                                robot.leftMotor.setPower(0.25);

                            }


                            break;

                        case "DRIVE":

                            cHdg = robot.getGyroFhdg() % 90;
                            hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                            curDistCount = (robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition()) / 2.0;

                            rDv = Range.clip(0.25 - hErr * 0.035, 0.15, 0.4);
                            lDv = Range.clip(0.25 + hErr * 0.035, 0.15, 0.4);

                            DbgLog.msg("SJH: /BEACON/DRIVE > r: %5.2f, l: %5.2f, d: %5.2f, a: %5.2f", rDv, lDv, drvTrn.countsToDistance(curDistCount), hErr );

                            robot.rightMotor.setPower(rDv);
                            robot.leftMotor.setPower(lDv);

                            if (bd.getBeaconPosZ() > 0.95) {
                                beaconStep = "WAIT";
                                robot.rightMotor.setPower(0);
                                robot.leftMotor.setPower(0);
                            }

                            break;

                    }

                    blueSide = bd.getBluePosSide();
                    redSide  = bd.getRedPosSide();

                    // Good when the beacon is in view enough or at least
                    // some driving done.
                    if ( pushSide == BeaconFinder.BeaconSide.UNKNOWN &&
                            blueSide != BeaconFinder.BeaconSide.UNKNOWN &&
                            redSide  != BeaconFinder.BeaconSide.UNKNOWN &&
                            beaconStep.equals("ALIGN") )
                    {
                        pushSide = redSide;

                        switch ( pushSide ){

                            case LEFT:
                                robot.lpusher.setPosition(0.85);
                                break;

                            case RIGHT:
                                robot.lpusher.setPosition(0.15);
                                break;
                        }

                        DbgLog.msg("SJH: /BEACON/FORWARD > FOUND ON %s", pushSide );

                    }

                    if ( drvTrn.countsToDistance( curDistCount ) > 5.0 && drvTrn.areDriveMotorsStuck() && drvTrn.isBusy() )
                    {
                        beaconStep = "WAIT";
                        robot.rightMotor.setPower(0);
                        robot.leftMotor.setPower(0);
                        DbgLog.msg("SJH: /BEACON/FORWARD > NOT MOVING? YIKES! %5.2f", curDistCount );
                    }
                }
            }

            sleep( 10 );
        }

        imgProc.stopSensing();
        cleanupCamera();
    }
}