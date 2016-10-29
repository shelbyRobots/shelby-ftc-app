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
public class LinearDetectColor extends LinearOpModeCamera {

    static {
        if (!OpenCVLoader.initDebug()) {
            DbgLog.error("SJH: OpenCVLoader error"); //Handle opencv loader issue
        }
    }

  Mat cvImage = null;

  int ds = 8;
  int ds2 = 2;  // additional downsampling of the image
  int cameraType = Camera.CameraInfo.CAMERA_FACING_FRONT;

  @Override
  public void runOpMode() throws InterruptedException {

    String colorString = "NONE";

    setCameraDownsampling(ds);
    Camera cam = initCamera(cameraType);
    if (cam != null)
    {
      cvImage = new Mat(width, height, CvType.CV_8UC1);

      telemetry.addLine("Wait for camera to finish initializing!");
      telemetry.update();
      startCamera(cameraType);  // can take a while.

      // best started before waitForStart
      telemetry.addLine("Camera ready!");
      telemetry.update();

      waitForStart();

      try { // try is needed so catch the interrupt when the opmode is ended to stop the camera
        while (opModeIsActive()) {
          if (imageReady()) {
            Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
            Utils.bitmapToMat(rgbImage, cvImage);

            BeaconFinder detector = new BeaconDetector(cvImage);

            DbgLog.msg("SH Beacon Color: " + detector.getLightOrder());
            telemetry.addData("Beacon Color: ", detector.getLightOrder());
            telemetry.update();
            sleep(10);
          }
        }
      } catch (Exception e) {
        stopCamera();
      }
    }
  }
}