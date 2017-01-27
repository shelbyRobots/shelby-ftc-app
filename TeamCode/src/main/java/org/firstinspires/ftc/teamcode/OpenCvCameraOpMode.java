package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.hardware.Camera;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import java.util.List;

public abstract class OpenCvCameraOpMode extends LinearOpMode
                                         implements CameraBridgeViewBase.CvCameraViewListener2
{
    protected boolean newImage = false;
    protected ImageProcessor imgProc = null;
    private static JavaCameraView openCVCamera = null;

    protected boolean flipImage = true;

    int width;
    int height;
    boolean initialized = false;

    boolean isInitialized() {
        return initialized;
    }

//    public void setCamera() {
//        if (openCVCamera == null)
//            return;
//        openCVCamera.disableView();
//        if (initialized) openCVCamera.disconnectCamera();
//        openCVCamera.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_BACK);
//        if (initialized)
//            if (!openCVCamera.connectCamera(width, height))
//                DbgLog.error("SJH Could not initialize camera!\r\n" +
//                              "This may occur because the OpenCV Manager is not installed,\r\n" +
//                              "CAMERA permission is not allowed in AndroidManifest.xml,\r\n" +
//                              "or because another app is currently locking it.");
//        openCVCamera.enableView();
//    }
//
//    public Size setFrameSize(Size frameSize) {
//        if (openCVCamera == null)
//            return null;
//
//        openCVCamera.disableView();
//        if (initialized) openCVCamera.disconnectCamera();
//        openCVCamera.setMaxFrameSize((int) frameSize.width, (int) frameSize.height);
//        if (initialized)
//            if (!openCVCamera.connectCamera((int) frameSize.width, (int) frameSize.height))
//                DbgLog.error("SJH: Could not initialize camera!\r\n" +
//                              "This may occur because the OpenCV Manager is not installed,\r\n" +
//                              "CAMERA permission is not allowed in AndroidManifest.xml,\r\n" +
//                              "or because another app is currently locking it.");
//        openCVCamera.enableView();
//
//        width = openCVCamera.getMeasuredWidth();
//        height = openCVCamera.getMeasuredHeight();
//        if (width == 0 || height == 0) {
//            DbgLog.msg("FTCVision", "OpenCV Camera failed to initialize width and height properties on startup.\r\n" +
//                                       "This is generally okay, but if you use width or height during init() you may\r\n" +
//                                       "run into a problem.");
//        }
//
//        return new Size(width, height);
//    }

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
                jcv.setMaxFrameSize(480, 320);

                ViewGroup.LayoutParams vglop =
                        new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT,
                                                   ViewGroup.LayoutParams.MATCH_PARENT);
                jcv.setLayoutParams(vglop);

                ViewGroup es = (ViewGroup) act.findViewById(R.id.entire_screen);
                ViewGroup tb = (ViewGroup) act.findViewById(R.id.top_bar);
                ViewGroup ih = (ViewGroup) act.findViewById(R.id.included_header);
                es.removeView(tb);
                es.removeView(ih);

                ViewGroup rl = (ViewGroup) act.findViewById(R.id.RelativeLayout);
                rl.removeViews(4,3);
                rl.removeViews(0,3);
                rl.setPadding(0,0,0,0);

                ViewGroup vg = (ViewGroup) act.findViewById(R.id.cameraMonitorViewId);

                vg.addView(jcv, 0);

                initialized = true;
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

        width = openCVCamera.getMeasuredWidth();
        height = openCVCamera.getMeasuredHeight();
        DbgLog.msg("SBH: CAMERA %d x %d", width, height);
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
