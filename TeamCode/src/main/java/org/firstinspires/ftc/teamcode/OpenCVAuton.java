package org.firstinspires.ftc.teamcode;

import android.app.Activity;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;

/**
 * Core OpMode class containing most OpenCV functionality
 */
@SuppressWarnings("WeakerAccess")
@Autonomous(name="OpenCVAuton", group ="Test")
public class OpenCVAuton extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    private JavaCameraView openCVCamera;

    private BeaconDetector bd = new BeaconDetector();
    private ShelbyBot   robot = new ShelbyBot();
    private Drivetrain drvTrn = new Drivetrain();

    private boolean useMotor  = true;
    private boolean gyroReady = false;
    private boolean follow    = false;

    public void runOpMode() {

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

        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS: {
                        openCVCamera.enableView();
                    }
                    break;
                    default: {
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }
        };

        openCVCamera = (JavaCameraView) ((Activity) hardwareMap.appContext).findViewById(R.id.surfaceView);
        openCVCamera.setVisibility(CameraBridgeViewBase.VISIBLE);
        openCVCamera.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_FRONT);
        openCVCamera.setCvCameraViewListener(this);

        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, hardwareMap.appContext, mLoaderCallback);
        } else {
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        waitForStart();

        BeaconDetector.BeaconSide blueSide = BeaconDetector.BeaconSide.UNKNOWN;
        BeaconDetector.BeaconSide redSide = BeaconDetector.BeaconSide.UNKNOWN;
        BeaconDetector.BeaconSide pushSide = BeaconDetector.BeaconSide.UNKNOWN;

        double baseSpeed = 0.4;

        double bConf, zPos, zOff, xPos, xOff, nPos = 0, rDv = 0, lDv = 0;
        double curDistCount = 0.0;

        double cRos = 0.03125;
        double dDist = 0;

        double cHdg;
        double hErr;

        String beaconStep = "WAIT";

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
                    if ( bd.getBeaconConf() > 0.35 && bd.getBeaconPosZ() < 0.7 ) {
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

                    switch (beaconStep) {
                        case "INIT":

                            blueSide = BeaconDetector.BeaconSide.UNKNOWN;
                            redSide = BeaconDetector.BeaconSide.UNKNOWN;
                            pushSide = BeaconDetector.BeaconSide.UNKNOWN;

                            robot.pusher.setPosition(0.1);

                            curDistCount = 0.0;
                            robot.gyro.resetZAxisIntegrator();
                            drvTrn.stopAndReset();

                            xPos = bd.getBeaconPosX();

                            nPos = xPos / 17.7;
                            dDist = 8.0 / Math.cos( Math.atan( nPos / 16.0 ) ) - 1.0;

                            rDv = Range.clip( baseSpeed + nPos * cRos * 1.8, 0.15, 0.65 );
                            lDv = Range.clip( baseSpeed - nPos * cRos * 1.8, 0.15, 0.65 );

                            DbgLog.msg("SJH: /BEACON/CENTER > r: %5.2f, l: %5.2f, xpos: %5.2f", rDv, lDv, xPos );

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

                                rDv = Range.clip( baseSpeed - nPos * cRos * 1.8, 0.15, 0.65 );
                                lDv = Range.clip( baseSpeed + nPos * cRos * 1.8, 0.15, 0.65 );

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

                            if (drvTrn.countsToDistance(curDistCount) > dDist * 2) {

                                beaconStep = "DRIVE";
                                robot.rightMotor.setPower(0.25);
                                robot.leftMotor.setPower(0.25);

                            }


                            break;

                        case "DRIVE":

                            cHdg = getGryoFhdg() % 90;
                            hErr = Math.abs( cHdg ) < 45 ? cHdg : Math.signum( cHdg ) * ( 90 - Math.abs( cHdg ) );

                            curDistCount = (robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition()) / 2.0;

                            rDv = Range.clip(0.25 - hErr * cRos, 0.15, 0.4);
                            lDv = Range.clip(0.25 + hErr * cRos, 0.15, 0.4);

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

        if (openCVCamera != null) {
            openCVCamera.disableView();
        }
    }

    private double getGryoFhdg()
    {
        double cHdg = robot.gyro.getIntegratedZValue();

        while (cHdg <= -180.0) cHdg += 360.0;
        while (cHdg >   180.0) cHdg -= 360.0;

        return cHdg;
    }

    public void onCameraViewStarted(int width, int height) {
        DbgLog.msg("SJH: CAMERA VIEW STARTED");
    }

    public void onCameraViewStopped() {
        DbgLog.msg("SJH: CAMERA VIEW STOPPED");
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat rgb = inputFrame.rgba();
        Mat flip = rgb.clone();

        Core.flip(rgb, flip, 1);
        bd.setImage( flip );

        return bd.drawBeacon();
    }
}