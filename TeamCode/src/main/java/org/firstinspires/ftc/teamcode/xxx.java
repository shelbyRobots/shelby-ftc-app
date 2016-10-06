//package org.firstinspires.ftc.teamcode;
//
//import FtcMenu;
//import hallib.FtcRobot;
//
//public class xxx extends FtcRobot
//{
//
//}

//
////package org.firstinspires.ftc.robotcontroller.internal.camMod;
//
//import android.graphics.Bitmap;
//import android.graphics.BitmapFactory;
//import android.graphics.ImageFormat;
//import android.graphics.Rect;
//import android.graphics.YuvImage;
//import android.hardware.Camera;
//import android.hardware.camera2.CameraDevice;
//import android.util.Log;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.vuforia.CameraCalibration;
//
//import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
//import org.firstinspires.ftc.robotcontroller.internal.camMod.archive.CameraPreview;
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//
//import java.io.ByteArrayOutputStream;
//import java.util.ArrayList;
//import java.util.List;
//
///**
// * Operations involving the camera on the phone.
// */
//public class CameraLib {
//    OpMode modeTeleOp;
//    //Color Sensing/////////////////////////////////////////////////////////////////////////////////
//    public Camera camera;
//    public CameraPreview preview;
//    public int width;
//    public int height;
//    public YuvImage yuvImage = null;
//    volatile private boolean imageReady = false;
//    private int looped = 0;
//    private String data;
//    private int ds = 1; // downsampling parameter
//    int rMask = 0xFF0000;
//    int gMask = 0xFF00;
//    int bMask = 0xFF;
//    ////////////////////////////////////////////////////
//    int ds2 = 1;  // additional downsampling of the image
//    private long lastLoopTime = 0;
//    Bitmap rgbImage;
//    int redValue, blueValue, greenValue;
//    float angleX, angleY, angleZ, posX, posY, posZ;
//    float[] dyson;
//    String colorString = "";
//    //Vuforia Related///////////////////////////////////////////////////////////////////////////////
//    VuforiaLocalizer vuforia;
//    OpenGLMatrix lastLocation = null;
//    OpenGLMatrix robotLocationTransform;
//    VuforiaTrackables robotParts;
//    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//    ////////////////////////////////////////////////////////////////////////////////////////////////
//    private String cameraMode = "INACTIVE";
//    private boolean cameraColorSensing = false;
//    private boolean cameraTracking = false;
//    public CameraLib(OpMode opMode) {
//        modeTeleOp = opMode;
//    }
//    /**General Functions***************************************************************************/
//    public void cameraDebugData() {
//        /**Debug Data******************************************************************************/
//        modeTeleOp.telemetry.addData("Cam Mode: ", getCameraMode());
//        if(cameraTracking) {
//            modeTeleOp.telemetry.addData("A ----------Vuforia Data----------", "");
//            modeTeleOp.telemetry.addData("B " + allTrackables.get(0).getName(), ((VuforiaTrackableDefaultListener)allTrackables.get(0).getListener()).isVisible() ? "Visible" : "Not Visible");
//            modeTeleOp.telemetry.addData("C " + allTrackables.get(1).getName(), ((VuforiaTrackableDefaultListener)allTrackables.get(1).getListener()).isVisible() ? "Visible" : "Not Visible");
//            modeTeleOp.telemetry.addData("D " + allTrackables.get(2).getName(), ((VuforiaTrackableDefaultListener)allTrackables.get(2).getListener()).isVisible() ? "Visible" : "Not Visible");
//            modeTeleOp.telemetry.addData("E " + allTrackables.get(3).getName(), ((VuforiaTrackableDefaultListener)allTrackables.get(3).getListener()).isVisible() ? "Visible" : "Not Visible");
//            if (lastLocation != null) {
//                modeTeleOp.telemetry.addData("F Pos", format(lastLocation));
//                modeTeleOp.telemetry.addData("G Sel Pos", "X: " + lastLocation.get(0, 3) + " Y: " + lastLocation.get(1, 3) + " Z: " + lastLocation.get(2, 3));
//                modeTeleOp.telemetry.addData("H Raw Data", lastLocation);
//                modeTeleOp.telemetry.addData("I Rows: ", lastLocation.numRows() + " Cols: " + lastLocation.numCols());
//                //You can find the angle by using math with the given vectors, the end.
//                modeTeleOp.telemetry.addData("J Sel Angles ",
//                        "X: " + 0 + " Y: " + Math.atan(lastLocation.get(2, 3)/lastLocation.get(1, 3)) + " Z: ");
//            }
//            else modeTeleOp.telemetry.addData("F Pos", "Unknown");
//        } else if(cameraColorSensing) {
//            modeTeleOp.telemetry.addData("A ----------Color Data----------", "");
//            modeTeleOp.telemetry.addData("B Color:", String.format("%s - %d, %d, %d", colorString, redValue, greenValue, blueValue));
//            modeTeleOp.telemetry.addData("C Width, height:", String.format("%d, %d", width, height));
//            modeTeleOp.telemetry.addData("D Image Ready", imageReady());
//            modeTeleOp.telemetry.addData("E RGB Image", rgbImage);
//        }
//    }
//    public void switchModes() {
//        if(modeTeleOp.gamepad1.y) {             //Switch to Vuforia Tracking
//            stopCamera();
//            vuforiaStartTracking();
//        } else if (modeTeleOp.gamepad1.x) {     //Switch to Camera Color Sensing
//            initCameraColorSensing();
//            vuforiaStopTracking();
//        } else if (modeTeleOp.gamepad1.b) {     //Disable Camera Operations
//            stopCamera();
//            vuforiaStopTracking();
//        }
//    }
//    public void runCameraOperations() {
//        if(cameraColorSensing) {
//            runCamera();
//        } else if(cameraTracking) {
//            vuforiaTracking();
//        }
//    }
//    public String getCameraMode() {
//        if(cameraColorSensing) return "COLOR_SENSING";
//        else if(cameraTracking) return "IMAGE_TRACKING";
//        else return "INACTIVE";
//    }
//    /**Camera Color Sensing Functions**************************************************************/
//    public Camera.PreviewCallback previewCallback = new Camera.PreviewCallback() {
//        public void onPreviewFrame(byte[] data, Camera camera) {
//            try {
//                Camera.Parameters parameters = camera.getParameters();
//                width = parameters.getPreviewSize().width;
//                height = parameters.getPreviewSize().height;
//                yuvImage = new YuvImage(data, ImageFormat.NV21, width, height, null);
//                imageReady = true;
//                looped += 1;
//            } catch (Exception e) {
//                e.printStackTrace();
//            }
//        }
//    };
//    private void setCameraDownsampling(int downSampling) {
//        ds = downSampling;
//    }
//    private boolean imageReady() {
//        return imageReady;
//    }
//    private boolean isCameraAvailable() {
//        int cameraId = -1;
//        Camera cam = null;
//        int numberOfCameras = Camera.getNumberOfCameras();
//        for (int i = 0; i < numberOfCameras; i++) {
//            Camera.CameraInfo info = new Camera.CameraInfo();
//            Camera.getCameraInfo(i, info);
//            if (info.facing == Camera.CameraInfo.CAMERA_FACING_FRONT) { // Camera.CameraInfo.CAMERA_FACING_FRONT or BACK
//                cameraId = i;
//                break;
//            }
//        }
//        try {
//            cam = Camera.open(cameraId);
//        } catch (Exception e) {
//            Log.e("Error", "Camera Not Available!");
//            return false;
//        }
//        cam.release();
//        cam = null;
//        return true;
//    }
//    private int red(int pixel) {
//        return ((pixel & rMask) >> 16);
//    }
//    private int green(int pixel) {
//        return ((pixel & gMask) >> 8);
//    }
//    private int blue(int pixel) {
//        return pixel & bMask;
//    }
//    private int highestColor(int red, int green, int blue) {
//        int[] color = {red, green, blue};
//        int value = 0;
//        for (int i = 1; i < 3; i++) {
//            if (color[value] < color[i])  value = i;
//        }
//        return value;
//    }
//    private Bitmap convertYuvImageToRgb(YuvImage yuvImage, int width, int height, int downSample) {
//        Bitmap rgbImage;
//        ByteArrayOutputStream out = new ByteArrayOutputStream();
//        yuvImage.compressToJpeg(new Rect(0, 0, width, height), 0, out);
//        byte[] imageBytes = out.toByteArray();
//
//        BitmapFactory.Options opt;
//        opt = new BitmapFactory.Options();
//        opt.inSampleSize = downSample;
//
//        rgbImage = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length, opt);
//        return rgbImage;
//    }
//    private Camera openCamera(int cameraInfoType) {
//        int cameraId = -1;
//        Camera cam = null;
//        int numberOfCameras = Camera.getNumberOfCameras();
//        for (int i = 0; i < numberOfCameras; i++) {
//            Camera.CameraInfo info = new Camera.CameraInfo();
//            Camera.getCameraInfo(i, info);
//            if (info.facing == cameraInfoType) { // Camera.CameraInfo.CAMERA_FACING_FRONT or BACK
//                cameraId = i;
//                break;
//            }
//        }
//        try {
//            cam = Camera.open(cameraId);
//        } catch (Exception e) {
//            Log.e("Error", "Can't Open Camera");
//        }
//        return cam;
//    }
//    private void startCamera() {
//        camera = openCamera(Camera.CameraInfo.CAMERA_FACING_BACK);
//        camera.setPreviewCallback(previewCallback);
//        Camera.Parameters parameters = camera.getParameters();
//        width = parameters.getPreviewSize().width / ds;
//        height = parameters.getPreviewSize().height / ds;
//        parameters.setPreviewSize(width, height);
//        camera.setParameters(parameters);
//        data = parameters.flatten();
//        if (preview == null) ((FtcRobotControllerActivity) modeTeleOp.hardwareMap.appContext).initPreview(camera, this, previewCallback);
//    }
//    public void initCameraColorSensing() {
//        if(!cameraColorSensing) {
//            startCamera();
//            setCameraDownsampling(1);
//            cameraColorSensing = true;
//        }
//    }
//    private void runCamera() {
//        if (imageReady()) { // only do this if an image has been returned from the camera
//            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
//
//            redValue = red(rgbImage.getPixel(width/2, height/2));
//            blueValue = blue(rgbImage.getPixel(width/2, height/2));
//            greenValue = green(rgbImage.getPixel(width/2, height/2));
//
//            switch (highestColor(redValue, greenValue, blueValue)) {
//                case 0:
//                    colorString = "RED";
//                    break;
//                case 1:
//                    colorString = "GREEN";
//                    break;
//                case 2:
//                    colorString = "BLUE";
//            }
//
//        }
//    }
//    public void stopCamera() {
//        if (camera != null) {
//            if (preview != null) {
//                ((FtcRobotControllerActivity) modeTeleOp.hardwareMap.appContext).removePreview();
//                preview = null;
//            }
//            camera.stopPreview();
//            camera.setPreviewCallback(null);
//            camera.release();
//            camera = null;
//            cameraColorSensing = false;
//        }
//    }
//    /**Vuforia Functions***************************************************************************/
//    public void vuforiaTracking() {
//        /**Tracking Data***************************************************************************/
//        robotLocationTransform = ((VuforiaTrackableDefaultListener)allTrackables.get(0).getListener()).getUpdatedRobotLocation();
//        if (robotLocationTransform != null) lastLocation = robotLocationTransform;
//        robotLocationTransform = ((VuforiaTrackableDefaultListener)allTrackables.get(1).getListener()).getUpdatedRobotLocation();
//        if (robotLocationTransform != null) lastLocation = robotLocationTransform;
//        robotLocationTransform = ((VuforiaTrackableDefaultListener)allTrackables.get(2).getListener()).getUpdatedRobotLocation();
//        if (robotLocationTransform != null) lastLocation = robotLocationTransform;
//        robotLocationTransform = ((VuforiaTrackableDefaultListener)allTrackables.get(3).getListener()).getUpdatedRobotLocation();
//        if (robotLocationTransform != null) lastLocation = robotLocationTransform;
//    }
//    public void vuforiaInitialization() {
//        //Vuforia Setup/////////////////////////////////////////////////////////////////////////////
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = "AXQq7X//////AAAAGcFjTEEHm0ELp5NwoHhmdEpNgTX8d6myjTcyrUlHChiJVJLIaIkK0OzHgJafo1JjkcMwYCNXJH3daOkuYutcLJbANx5S8zikC4ofJ2r83WJQYD1lBWl1Ek8sKyUizD+op9OsestgiRJoQ8r4U6JoM7TReghbdzBIfyPZv9R6ltI+4PcIouK5odsQw/HQz0BuYmH3n4RO96Af8Oo+uoNbnx9/9vWG3R0js4y0+nujH19lkQSdqdN1IeQoeeMa6bCsAI9h0JjsGfACdX8sHYnhvyueL7NHpM1mcME5DttS826jRUIqrTjf60eHwFZdVvWgyVbalKRwrcZRsYQsO+lesWr6Rhd0HmFMrfkQ0I7cUXJ3";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//        CameraCalibration cali = vuforia.getCameraCalibration();
//        //Target Registration///////////////////////////////////////////////////////////////////////
//        robotParts = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
//        VuforiaTrackable wheelTarget = robotParts.get(0);                                           //Wheels
//        wheelTarget.setName("Wheels");
//        VuforiaTrackable toolTarget  = robotParts.get(1);                                           //Tools
//        toolTarget.setName("Tools");
//        VuforiaTrackable legoTarget = robotParts.get(2);                                            //Legos
//        legoTarget.setName("Legos");
//        VuforiaTrackable gearTarget = robotParts.get(3);                                            //Gears
//        gearTarget.setName("Gears");
//        allTrackables.addAll(robotParts);
//        /**Image Field Location Settings***********************************************************/
//        //Target 1: Wheels /////////////////////////////////////////////////////////////////////////
//        OpenGLMatrix wheelTargetLocationMatrix = OpenGLMatrix
//                .translation(0, 0, 0)
//                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 0, 0, 0));
//        wheelTarget.setLocation(wheelTargetLocationMatrix);
//        //Target 2: Tools ///////////////////////////////////////////////////////////////////////////
//        OpenGLMatrix toolTargetLocationMatrix = OpenGLMatrix
//                .translation(0, 0, 0)
//                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 0, 0, 0));
//        toolTarget.setLocation(toolTargetLocationMatrix);
//        //Target 3: Legos //////////////////////////////////////////////////////////////////////////
//        OpenGLMatrix legoTargetLocationMatrix = OpenGLMatrix
//                .translation(0, 0, 0)
//                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 0, 0, 0));
//        legoTarget.setLocation(legoTargetLocationMatrix);
//        //Target 4: Gears //////////////////////////////////////////////////////////////////////////
//        OpenGLMatrix gearTargetLocationMatrix = OpenGLMatrix
//                .translation(0, 0, 0)
//                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ,
//                        AngleUnit.DEGREES, 0, 0, 0));
//        gearTarget.setLocation(gearTargetLocationMatrix);
//        //Phone's location on Robot/////////////////////////////////////////////////////////////////
//        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
//                .translation(0,0,0)
//                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZY,
//                        AngleUnit.DEGREES, 0, 0, 0));
//        /**Listeners*******************************************************************************/
//        ((VuforiaTrackableDefaultListener)wheelTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        ((VuforiaTrackableDefaultListener)toolTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        ((VuforiaTrackableDefaultListener)legoTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        ((VuforiaTrackableDefaultListener)gearTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//    }
//    public void vuforiaStartTracking() {
//        vuforiaInitialization();
//        if(!cameraTracking) {
//            robotParts.activate();
//        }
//        cameraTracking = true;
//    }
//    public void vuforiaStopTracking() {
//        if(cameraTracking) {
//            robotParts.deactivate();
//            this.vuforia = null;
//        }
//        cameraTracking = false;
//    }
//    String format(OpenGLMatrix transformationMatrix) {
//        return transformationMatrix.formatAsTransform();
//    }
//}
