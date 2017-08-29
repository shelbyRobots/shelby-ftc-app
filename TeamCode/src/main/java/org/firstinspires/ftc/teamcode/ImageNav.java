/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * This OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@SuppressWarnings("WeakerAccess")
@Autonomous(name="ImageNav", group ="Concept")
//@Disabled
public class ImageNav extends LinearOpMode {

    private static final String TAG = "SJH Image Tracker";

    // Vuforia units are mm = units used in XML for the trackables

    private static final float MM_PER_INCH        = 25.4f;

    private VectorF currPos = new VectorF(0.0f, 0.0f, 0.0f);
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable blueWheels;
    private VuforiaTrackable blueLegos;
    private VuforiaTrackable redTools;
    private VuforiaTrackable redGears;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables ftcImages;
    private String lastVisName = "UNKNOWN";
    private boolean useScreen = true;

    private Bitmap rgbImage = null;
    private BeaconDetector detector = new BeaconDetector();

    private ElapsedTime timer = new ElapsedTime();

    public void setupTrackables()
    {
        //To see camera feedback, pass the view id
        //For competition, we don't want this - so use the no param ctor
        if(useScreen)
        {
            parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        }
        else
        {
            parameters = new VuforiaLocalizer.Parameters();
        }

        //SJH Teams license
        parameters.vuforiaLicenseKey =
                "AQgIvJ7/////AAAAGQSociXWO0kDvfP15zd4zOsS+fHJygDMLA" +
                        "1HhOJQ3FkeiPLGU6YW3ru+jzC6MGxM5tY1ajF4Y0plOpxhQGfS" +
                        "R4g3zFiP0IQavezWhGbjBCRMmYu8INy8KvoZ03crZe9wxxQJu9" +
                        "9KiNX3ZrbUevNXODKKzWyA9RqxxQHbJ3gpXoff4z1O9n211VOg" +
                        "EsJjrNZq8xJnznilyXwc8colJnZD/Adr6UmOzxoUGgaMrdPrlj" +
                        "McDJZU6uyoIrOjiv1G2r3iNjtd7LzKAANKrK/0IrO90MgRqQDr" +
                        "CAAJVHqqyyubMy8EqE5onzw/WFEcEwfQ6nolsNwYTEZb/JppU8" +
                        "9Q6DZmhz4FCT49shA+4PyNOzqsjhRC";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat( PIXEL_FORMAT.RGB565, true );

        RobotLog.ii("SJH", "Vuforia LicKey: " + parameters.vuforiaLicenseKey);

        ftcImages = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        //Wheels are on blue side closest to blue corner
        blueWheels = ftcImages.get(0);
        blueWheels.setName("BlueWheels");

        //Legos are on blud side furthest from blue corner
        blueLegos = ftcImages.get(2);
        blueLegos.setName("BlueLegos");

        //Tools are on red side furthest from red corner
        redTools = ftcImages.get(1);
        redTools.setName("RedTools");

        //Gears are on red side closest to red corner
        redGears = ftcImages.get(3);
        redGears.setName("RedGears");

        allTrackables.addAll(ftcImages);

        redTools.setLocation(Field.redToolsLocationOnField);
        RobotLog.ii(TAG, "Red Tools=%s", format(Field.redToolsLocationOnField));

        redGears.setLocation(Field.redToolsLocationOnField);
        RobotLog.ii(TAG, "Red Gears=%s", format(Field.redGearsLocationOnField));

        blueWheels.setLocation(Field.blueWheelsLocationOnField);
        RobotLog.ii(TAG, "Blue Wheels=%s", format(Field.blueWheelsLocationOnField));

        blueLegos.setLocation(Field.blueLegosLocationOnField);
        RobotLog.ii(TAG, "Blue Legos=%s", format(Field.blueLegosLocationOnField));
    }

    public void setupPhoneOnRobot()
    {
        OpenGLMatrix phoneLocationOnRobot = ShelbyBot.phoneLocationOnRobot;
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot     maps   phone coords        -> robot coords
         * P = tracker.getPose()        maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()                 maps   robot coords -> phone coords
         * P.inverted()                 maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        ((VuforiaTrackableDefaultListener)redTools.getListener()).
                                                                         setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redGears.getListener()).
                                                                         setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueLegos.getListener()).
                                                                          setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueWheels.getListener()).
                                                                           setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
    }


    public OpenGLMatrix getRobotLocation()
    {
        /**
         * getUpdatedRobotLocation() will return null if no new information is available
         * since the last time that call was made, or if the trackable is not currently
         * visible.
         * getRobotLocation() will return null if the trackable is not currently visible.
         */
        OpenGLMatrix robotLocationTransform = null;
        for (VuforiaTrackable trackable : allTrackables)
        {
            robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener())
                                             .getUpdatedRobotLocation();
            if(robotLocationTransform != null)
            {
                lastVisName = trackable.getName();
                break;
            }
        }
        return robotLocationTransform;
    }

    public String getLocStirng(OpenGLMatrix mat)
    {
        String locStr = null;
        if(mat != null)
        {
            float xyz[] = mat.getTranslation().getData();
            Orientation ori = Orientation.getOrientation(lastLocation,
                    AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

            locStr = String.format(Locale.US,
                    "%10s POS: %5.2f, %5.2f, %5.2f ROT: %4.1f, %4.1f, %4.1f",
                    lastVisName, xyz[0] / MM_PER_INCH, xyz[1] / MM_PER_INCH, xyz[2] / MM_PER_INCH,
                    ori.firstAngle, ori.secondAngle, ori.thirdAngle);
        }
        return locStr;
    }

    public Bitmap getImage()
    {
        VuforiaLocalizer.CloseableFrame frame = null;
        try
        {
            frame = vuforia.getFrameQueue().take();
        }
        catch (InterruptedException e)
        {
            RobotLog.ii("SJH", "What is going on here");
        }

        if(frame == null) return null;
        long numImages = frame.getNumImages();

        Image imgdata = null;
        for (int i = 0; i < numImages; i++)
        {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565)
            {
                imgdata = frame.getImage(i);
                break;
            }
        }

        //frame.close();
        if(imgdata == null) return null;

        int imgW = imgdata.getWidth();
        int imgH = imgdata.getHeight();
        Bitmap.Config imgT = Bitmap.Config.RGB_565;
        if(rgbImage == null) rgbImage = Bitmap.createBitmap(imgW, imgH, imgT);
        rgbImage.copyPixelsFromBuffer(imgdata.getPixels());

        return rgbImage;
    }

    @Override
    public void runOpMode()
    {
        setupTrackables();
        setupPhoneOnRobot();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        waitForStart();


        timer.reset();

        telemetry.addData(":", "Visual Cortex activated!");
        RobotLog.ii("SJH", "Visual Cortex activated!");
        telemetry.update();

        ftcImages.activate();
        OpenGLMatrix robotLocationTransform;
        while (opModeIsActive())// && timer.seconds() < 100
        {
            /** Start tracking the data sets we care about. */
            robotLocationTransform = getRobotLocation();
            if (robotLocationTransform != null)
            {
                lastLocation = robotLocationTransform;
                currPos = lastLocation.getTranslation();
                String locStr = getLocStirng(lastLocation);
                telemetry.addData("LOC", locStr);
                RobotLog.ii("SJH", locStr);
            }

            vuforia.setFrameQueueCapacity(10);
            Bitmap rgbImage = getImage();

            if(rgbImage == null) continue;
            detector.setBitmap(rgbImage);
            vuforia.setFrameQueueCapacity(0);

            RobotLog.ii("SJH", "Beacon Color: " + detector.getLightOrder());
            telemetry.addData("Beacon Color: ", detector.getLightOrder());

            telemetry.update();
            idle();
        }
        ftcImages.deactivate();

    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix)
    {
        return transformationMatrix.formatAsTransform();
    }
}
