package sample_camera_opmodes;

import android.graphics.Bitmap;
import android.hardware.Camera;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BeaconDetector;
import org.firstinspires.ftc.teamcode.BeaconFinder;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import for_camera_opmodes.LinearOpModeCamera;

@SuppressWarnings({"unused", "WeakerAccess", "deprecation"})
@TeleOp(name = "LinearDetectColor", group = "ZZOpModeCameraPackage")
//@Disabled
public class LinearDetectColor extends LinearOpModeCamera
{

    static {
        if (!OpenCVLoader.initDebug()) {
            DbgLog.error("SJH: OpenCVLoader error"); //Handle opencv loader issue
        }
    }

  Mat cvImage = null;
  int convertDownSample = 2;  // additional downsampling of the image

  private void setupCamera()
  {
    int cameraDownSample  = 1;
    int cameraType = Camera.CameraInfo.CAMERA_FACING_FRONT;

    setCameraDownsampling(cameraDownSample);
    Camera cam = initCamera(cameraType);
    if (cam != null)
    {
      cvImage = new Mat(width, height, CvType.CV_8UC1);

      DbgLog.msg("Wait for camera to finish initializing!");
      startCamera(cameraType);  // can take a while.

      // best started before waitForStart
      DbgLog.msg("Camera ready!");
    }
  }

  private void procImage()
  {
    if (imageReady())
    {
      Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height,
                                             convertDownSample);
      Utils.bitmapToMat(rgbImage, cvImage);

      BeaconFinder detector = new BeaconDetector(cvImage);

      DbgLog.msg("SJH Beacon Color: " + detector.getLightOrder());
      telemetry.addData("Beacon Color: ", detector.getLightOrder());
      telemetry.update();
      sleep(10);
    }
  }

  @Override
  public void runOpMode() throws InterruptedException
  {
    setupCamera();
    waitForStart();

    try
    { // try is needed so catch the interrupt when the opmode is ended to stop the camera
      while (opModeIsActive()) procImage();
    }
    catch (Exception e)
    {
      stopCamera();
    }
  }
}