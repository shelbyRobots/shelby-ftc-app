package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.view.ViewGroup;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;

public abstract class OpenCvCameraOpMode extends LinearOpMode
                                         implements CameraBridgeViewBase.CvCameraViewListener2
{
    protected boolean newImage = false;
    protected ImageProcessor imgProc = null;
    private static JavaCameraView openCVCamera = null;

    protected boolean flipImage = true;

    private void setupCameraView()
    {
        final Activity act = (Activity) hardwareMap.appContext;
        act.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        class CameraRunnable implements Runnable
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

        CameraRunnable mr = new CameraRunnable();
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

    protected void initOpenCv()
    {
        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext)
        {
            @Override
            public void onManagerConnected(int status)
            {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS:
                    {
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

    protected void cleanupCamera()
    {
        if (openCVCamera != null)
        {
            openCVCamera.disableView();
            openCVCamera.disconnectCamera();
        }
    }

    protected void setImageProcessor(ImageProcessor imgProc)
    {
        this.imgProc = imgProc;
    }

    protected ImageProcessor getImageProcessor() { return imgProc; }

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        DbgLog.msg("SJH: CAMERA VIEW STARTED %4dx%4d", width, height);
    }

    @Override
    public void onCameraViewStopped()
    {
        DbgLog.msg("SJH: CAMERA VIEW STOPPED");
        if(imgProc != null)
        {
            imgProc.stopSensing();
        }

        if (openCVCamera != null)
        {
            openCVCamera.disableView();
        }
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame)
    {
        Mat rgb = inputFrame.rgba();
        newImage = true;

        Mat rtrnImage = rgb;

        if(imgProc != null)
        {
            if(flipImage)
            {
                Mat flip = rgb.clone();
                Core.flip(rgb, flip, 1);
                imgProc.setImage(flip);
            }
            else
            {
                imgProc.setImage( rgb );
            }

            rtrnImage = imgProc.draw();
        }

        return rtrnImage;
    }
}
