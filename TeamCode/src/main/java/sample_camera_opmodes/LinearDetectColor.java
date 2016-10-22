package sample_camera_opmodes;

import android.graphics.Bitmap;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BeaconDetector;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name = "LinearDetectColor", group = "ZZOpModeCameraPackage")
//@Disabled
public class LinearDetectColor extends LinearOpModeCamera {

    static {
        if (!OpenCVLoader.initDebug()) {
            // Handle initialization error
        }
    }

  DcMotor motorRight;
  DcMotor motorLeft;
  Mat cvImage = null;

  int ds2 = 2;  // additional downsampling of the image
  // set to 1 to disable further downsampling

  @Override
  public void runOpMode() throws InterruptedException {

    String colorString = "NONE";

    // linear OpMode, so could do stuff like this too.
        /*
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        */

    if (isCameraAvailable()) {

      setCameraDownsampling(8);
      // parameter determines how downsampled you want your images
      // 8, 4, 2, or 1.
      // higher number is more downsampled, so less resolution but faster
      // 1 is original resolution, which is detailed but slow
      // must be called before super.init sets up the camera

      telemetry.addLine("Wait for camera to finish initializing!");
      telemetry.update();
      startCamera();  // can take a while.

      cvImage = new Mat(width, height, CvType.CV_8UC1);

        // best started before waitForStart
      telemetry.addLine("Camera ready!");
      telemetry.update();

      waitForStart();

      // LinearOpMode, so could do stuff like this too.
        /*
        motorLeft.setPower(1);  // drive forward
        motorRight.setPower(1);
        sleep(1000);            // for a second.
        motorLeft.setPower(0);  // stop drive motors.
        motorRight.setPower(0);
        sleep(1000);            // wait a second.
        */
      try { // try is needed so catch the interrupt when the opmode is ended to stop the camera
        while (opModeIsActive()) {
          if (imageReady()) { // only do this if an image has been returned from the camera

            Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

            Utils.bitmapToMat(rgbImage, cvImage);

            BeaconDetector detector = new BeaconDetector(cvImage);

            DbgLog.msg("SH:" + "Red? " + String.valueOf(detector.isRed()));
            DbgLog.msg("SH:" + "Blue? " + String.valueOf(detector.isBlue()));
            telemetry.addData("Color:", "Red? " + String.valueOf(detector.isRed()));
            telemetry.addData("Color:", "Blue? " + String.valueOf(detector.isBlue()));
            telemetry.update();
            sleep(10);
          }
        }
      } catch (Exception e) {
        stopCamera();
        throw e;
      }
    }
  }
}
