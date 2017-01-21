package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.view.View;
import android.view.ViewGroup;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

/**
 * Core OpMode class containing most OpenCV functionality
 */
@SuppressWarnings("WeakerAccess")
@Autonomous(name="OpenCvStandAlone", group ="Test")
public class OpenCvStandAlone extends LinearOpMode
                              implements CameraBridgeViewBase.CvCameraViewListener2
{
    private JavaCameraView openCVCamera;

    private LedDetector ld = new LedDetector();

    private boolean newImage = false;

    public void setupCameraView()
    {
        final Activity act = (Activity) hardwareMap.appContext;
        act.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        //Alternatively, create jcv in code

        class MyRun implements Runnable
        {
            private JavaCameraView jcv = null;

            public JavaCameraView getJcv() { return jcv; }

            public void run() {
                jcv = new JavaCameraView(act, CameraBridgeViewBase.CAMERA_ID_BACK);
                ViewGroup.LayoutParams vglop =
                        new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT,
                                                   ViewGroup.LayoutParams.MATCH_PARENT);
                jcv.setLayoutParams(vglop);
                ViewGroup vg = (ViewGroup) act.findViewById(R.id.cameraMonitorViewId);
                vg.addView(jcv);
            }
        }

        MyRun mr = new MyRun();
        act.runOnUiThread(mr);

        JavaCameraView jcv = null;
        while(jcv == null)
        {
            jcv = mr.getJcv();
        }

        openCVCamera = jcv;

        openCVCamera.setVisibility(CameraBridgeViewBase.VISIBLE);
        openCVCamera.setCvCameraViewListener(this);
    }

    private void initOpenCv()
    {
        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext)
        {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS: {
                        setupCameraView();
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

        if (!OpenCVLoader.initDebug())
        {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0,
                    hardwareMap.appContext,
                    mLoaderCallback);
        } else
        {
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public void runOpMode()
    {
        initOpenCv();

        waitForStart();

        ld.startSensing();

        while(opModeIsActive())
        {
            if(newImage)
            {
                newImage = false;
                ld.logDebug();
                telemetry.addData("#LEDs", "%4d", ld.getNumLEDs());
                telemetry.update();
            }
        }

        ld.stopSensing();

        if (openCVCamera != null) {
            openCVCamera.disableView();
        }
    }

    public void onCameraViewStarted(int width, int height) {
        DbgLog.msg("SJH: CAMERA VIEW STARTED");
    }

    public void onCameraViewStopped()
    {
        DbgLog.msg("SJH: CAMERA VIEW STOPPED");
        if (openCVCamera != null)
        {
            openCVCamera.disableView();
        }
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat rgb = inputFrame.rgba();
        newImage = true;

        ld.setImage( rgb );

        return ld.draw();
    }
}
