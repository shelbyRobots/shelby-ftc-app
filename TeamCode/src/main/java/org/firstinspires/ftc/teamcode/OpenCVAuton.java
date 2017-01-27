package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Core;
import org.opencv.core.Mat;

/**
 * Core OpMode class containing most OpenCV functionality
 */
@SuppressWarnings("WeakerAccess")
@Autonomous(name="OpenCVAuton", group ="Test")
public class OpenCVAuton extends VisionOpModeCore {

    private BeaconDetector bd = new BeaconDetector();
    private ShelbyBot   robot = new ShelbyBot();
    private Drivetrain drvTrn = new Drivetrain();

    private boolean useMotor  = false;
    private boolean gyroReady = false;
    private boolean follow    = false;

    @Override
    public void initRobot()
    {
        if ( useMotor ) {
            robot.init(hardwareMap);

            drvTrn.init(robot.leftMotor, robot.rightMotor, robot.gyro);

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
                if(gyroTimer.seconds() > gyroInitTimout)
                {
                    DbgLog.msg("SJH: GYRO INIT TIMED OUT!!");
                    gyroCalibTimedout = true;
                    break;
                }
            }
            DbgLog.msg("SJH: Gyro callibrated in %4.2f seconds", gyroTimer.seconds());

            gyroReady = !gyroCalibTimedout;
            if(gyroReady) robot.gyro.resetZAxisIntegrator();

            drvTrn.setGryoReady(gyroReady);
        }

        initVision();
    }

    @Override
    public void startMode()
    {
        BeaconDetector.BeaconSide blueSide = BeaconDetector.BeaconSide.UNKNOWN;
        BeaconDetector.BeaconSide redSide = BeaconDetector.BeaconSide.UNKNOWN;
        BeaconDetector.BeaconSide pushSide = BeaconDetector.BeaconSide.UNKNOWN;

        double baseSpeed = 0.4;

        double bConf, zPos, zOff, xPos, xOff, nPos = 0, rDv = 0, lDv = 0;
        double tPow1 = 0, tPow2 = 0, nAng = 0, dDist = 0;
        double curDistCount = 0.0;
        double cHdg, hErr, nOff;

        String beaconStep = "WAIT";

        if ( useMotor )
            robot.gyro.resetZAxisIntegrator();

        startVision();
        bd.startSensing();

        sleep( 200 );

        while(opModeIsActive()) {

            bd.logDebug();

            telemetry.addData( "CONF", "%5.2f", bd.getBeaconConf() );
            telemetry.addData( "X", "%5.2f", bd.getBeaconPosX() );
            telemetry.addData( "Z", "%5.2f", bd.getBeaconPosZ() );
            telemetry.addData( "RED", "%s", bd.getRedPosSide() );
            telemetry.addData( "BLUE", "%s", bd.getBluePosSide() );
            telemetry.addData( "RDV", "%5.2f", rDv );
            telemetry.addData( "LDV", "%5.2f", lDv );
            telemetry.addData( "DIST", "%5.2f", (double) drvTrn.countsToDistance(curDistCount) );

            telemetry.update();

            if (useMotor)
            {
                if ( follow ) {
                    if ( bd.getBeaconConf() > 0.25 && bd.getBeaconPosZ() < 0.7 ) {
                        zOff = 1.0 - bd.getBeaconPosZ();
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

                            blueSide = BeaconDetector.BeaconSide.UNKNOWN;
                            redSide = BeaconDetector.BeaconSide.UNKNOWN;
                            pushSide = BeaconDetector.BeaconSide.UNKNOWN;

                            robot.pusher.setPosition(0.1);

                            curDistCount = 0.0;
                            drvTrn.stopAndReset();

                            cHdg = getGryoFhdg() % 90;
                            hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                            xPos = bd.getBeaconPosX();

                            nOff = 24.0 * Math.tan( Math.toRadians( hErr ) );
                            nPos = xPos / 17.7; // + nOff;
                            nAng = Math.atan( nPos / 9.0 );
                            dDist = 6.0 / Math.cos( nAng );
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

                            cHdg = getGryoFhdg() % 90;
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

                            cHdg = getGryoFhdg() % 90;
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

                            cHdg = getGryoFhdg() % 90;
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
                    redSide = bd.getRedPosSide();

                    // Good when the beacon is in view enough or at least
                    // some driving done.
                    if ( pushSide == BeaconDetector.BeaconSide.UNKNOWN &&
                            blueSide != BeaconDetector.BeaconSide.UNKNOWN &&
                            redSide != BeaconDetector.BeaconSide.UNKNOWN &&
                            beaconStep == "ALIGN" )
                    {
                        pushSide = redSide;

                        switch ( pushSide ){

                            case LEFT:
                                robot.pusher.setPosition(0.85);
                                break;

                            case RIGHT:
                                robot.pusher.setPosition(0.15);
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

        bd.stopSensing();
        stopVision();
    }

    private double getGryoFhdg()
    {
        double cHdg = robot.gyro.getIntegratedZValue();

        while (cHdg <= -180.0) cHdg += 360.0;
        while (cHdg >   180.0) cHdg -= 360.0;

        return cHdg;
    }

    public Mat processFrame(Mat rgba, Mat gray) {

        Mat flip = rgba.clone();
        Core.flip(rgba, flip, 1);
        bd.setImage( rgba );

        return bd.drawBeacon();
    }

}