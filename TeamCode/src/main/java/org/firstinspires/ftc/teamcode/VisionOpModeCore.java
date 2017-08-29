package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Size;

/**
 * Created by crazy on 1/21/2017.
 */

public abstract class VisionOpModeCore extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    private static final int initialMaxSize = 1000;
    public static JavaCameraView openCVCamera;
    private static boolean initialized = false;
    private static boolean openCVInitialized = false;
    public int width, height;

    public VisionOpModeCore() {
        initialized = false;
        openCVCamera = null;
    }

    boolean isInitialized() {
        return initialized;
    }

    private void error(String message) {
        RobotLog.ii("SJH", "%s", message);
        telemetry.addData("Vision Status", message);
    }

    /**
     * Set the maximum frame size that the camera uses
     * This method will fail if the camera is locked - it is recommended to check the result.
     *
     * @param frameSize Maximum (target) frame size
     * @return Actual frame size or null if cannot be set
     */
    public Size setFrameSize(Size frameSize) {
        if (openCVCamera == null)
            return null;

        openCVCamera.disableView();
        if (initialized) openCVCamera.disconnectCamera();
        openCVCamera.setMaxFrameSize((int) frameSize.width, (int) frameSize.height);

        if (initialized)
            if (!openCVCamera.connectCamera((int) frameSize.width, (int) frameSize.height))
                error("SJH: Could not initialize camera!\r\n" +
                        "This may occur because the OpenCV Manager is not installed,\r\n" +
                        "CAMERA permission is not allowed in AndroidManifest.xml,\r\n" +
                        "or because another app is currently locking it.");


        openCVCamera.enableView();

        width = openCVCamera.getWidth();
        height = openCVCamera.getHeight();
        if (width == 0 || height == 0) {
            RobotLog.ii("SJH", "%s", "OpenCV Camera failed to initialize width and height properties on startup.\r\n" +
                    "This is generally okay, but if you use width or height during init() you may\r\n" +
                    "run into a problem.");
        }

        return new Size(width, height);
    }

    /**
     * Get the actual frame size
     *
     * @return Actual frame size in pixels
     */
    public Size getFrameSize() {
        return new Size(width, height);
    }

    public void initVision() {
        //Initialize camera view
        if ( initialized ) return;

        BaseLoaderCallback openCVLoaderCallback = null;
        try {
            openCVLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
                @Override
                public void onManagerConnected(int status) {
                    switch (status) {
                        case LoaderCallbackInterface.SUCCESS: {
                            //Woohoo!
                            RobotLog.ii("SJH", "OpenCV Manager connected!");
                            openCVInitialized = true;
                        }
                        break;
                        default: {
                            super.onManagerConnected(status);
                        }
                        break;
                    }
                }
            };
        } catch (NullPointerException e) {
            error("Could not find OpenCV Manager!\r\n" +
                    "Please install the app from the Google Play Store.");
        }

        final Activity activity = (Activity) hardwareMap.appContext;
        final VisionOpModeCore t = this;

        activity.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        if (!OpenCVLoader.initDebug()) {
            RobotLog.ii("SJH", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            boolean success = OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, hardwareMap.appContext, openCVLoaderCallback);
            if (!success) {
                error("Could not initialize OpenCV!\r\n" +
                        "Did you install the OpenCV Manager from the Play Store?");
            } else {
                RobotLog.ii("SJH", "OpenCV: Asynchronous initialization succeeded!");
            }
        } else {
            RobotLog.ii("SJH", "OpenCV library found inside package. Using it!");
            if (openCVLoaderCallback != null)
                openCVLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
            else {
                RobotLog.ee("SJH", "Failed to load OpenCV from package!");
                return;
            }
        }

        while (!openCVInitialized) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {

                ViewGroup layout = (ViewGroup) activity.findViewById(R.id.RelativeLayout);
                openCVCamera = new JavaCameraView(hardwareMap.appContext, CameraBridgeViewBase.CAMERA_ID_BACK);

                openCVCamera.setLayoutParams(new LinearLayout.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));

                layout.addView(openCVCamera);
                layout.setVisibility(View.VISIBLE);

                openCVCamera.setMaxFrameSize((int) initialMaxSize, (int) initialMaxSize);

                width = openCVCamera.getWidth();
                height = openCVCamera.getHeight();
                initialized = true;

            }
        });

        while (!initialized) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }



    }

    public void startVision() {
        if (openCVCamera != null) {
            openCVCamera.setCvCameraViewListener( this );
            openCVCamera.enableView();
        }
    }
    public void stopVision() {
        if (openCVCamera != null) {
            openCVCamera.disableView();
        }
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        this.width = width;
        this.height = height;
        RobotLog.ii("SJH", "size " + String.valueOf(width) + "x" + String.valueOf(height));
        RobotLog.ii("SJH", "CAMERA STARTED");
    }

    @Override
    public void onCameraViewStopped() {
        RobotLog.ii("SJH", "CAMERA STOPPED");
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        if (!initialized) {
            return inputFrame.rgba();
        }

        return processFrame(inputFrame.rgba(), inputFrame.gray());
    }

    public abstract Mat processFrame(Mat rgba, Mat gray);
}
