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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

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
@Autonomous(name="ImageAuton", group ="Test")
//@Disabled
public class ImageAuton extends LinearOpMode {

    private static final String TAG = "SJH Image Tracker";

    // Vuforia units are mm = units used in XML for the trackables

    private static final float MM_PER_INCH        = 25.4f;

    private VectorF currPos = new VectorF(0.0f, 0.0f, 0.0f);
    private OpenGLMatrix lastLocation = null;

    private BeaconDetector bd = new BeaconDetector();
    private ImageTracker tracker = new ImageTracker();

    private ElapsedTime timer = new ElapsedTime();
    private Point2d curPos;
    private double curHdg;

    private boolean findSensedLoc()
    {
        RobotLog.ii("SJH", "findSensedLoc");
        tracker.setActive(true);
        telemetry.addData("2", "STATE: %s", "FIND IMG LOC");
        curPos = null;
        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        tracker.setActive(true);
        Point2d sensedBotPos = null;
        double  sensedFldHdg = 90.0;
        OpenGLMatrix robotLocationTransform;
        int frm = 0;
        while(sensedBotPos == null && itimer.milliseconds() < 1000)
        {
            tracker.updateRobotLocationInfo();
            sensedBotPos = tracker.getSensedPosition();

            if(sensedBotPos != null)
            {
                curPos = sensedBotPos;
                sensedFldHdg = tracker.getSensedFldHeading();
                curHdg = sensedFldHdg;
                tracker.setActive(false);
            }
            else
            {
                sleep(50);
            }
            frm++;
        }

        if ( sensedBotPos != null )
        {
            double t = itimer.seconds();
            RobotLog.ii("SJH", "Senesed Pos: %s %5.2f %2.3f", sensedBotPos, sensedFldHdg, t);
            RobotLog.ii("SJH", "IMG %s frame %d", tracker.getLocString(), frm);
            telemetry.addData("SLOC", "SLOC: %s %4.1f", sensedBotPos, sensedFldHdg);
            telemetry.addData("IMG", "%s  frame %d", tracker.getLocString(), frm);
        }
        else
        {
            telemetry.addData("4", "SENSLOC: %s", "NO VALUE");
        }

//        robotLocationTransform = tracker.getRobotLocation();
//        if (robotLocationTransform != null)
//        {
//            lastLocation = robotLocationTransform;
//            currPos = lastLocation.getTranslation();
//            String locStr = tracker.getLocString();
//            telemetry.addData("LOC2", locStr);
//            RobotLog.ii("SJH", " + locStr);
//        }

        return (curPos != null);
    }

    private void do_findBeaconOrder(boolean push)
    {
        RobotLog.ii("SJH", "FIND BEACON ORDER!!!");
        telemetry.addData("2", "STATE: %s", "BEACON FIND");
        int timeout = 500;
        BeaconFinder.LightOrder ord = BeaconFinder.LightOrder.UNKNOWN;
        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        tracker.setFrameQueueSize(10);
        tracker.setActive(true);
        int frame = 0;
        while ((ord == BeaconFinder.LightOrder.UNKNOWN  ||
                ord == BeaconFinder.LightOrder.RED_RED  ||
                ord == BeaconFinder.LightOrder.BLUE_BLUE) &&
                itimer.milliseconds() < timeout)
        {
            Bitmap bmap = tracker.getImage();
            if (bmap != null)
            {
                bd.setBitmap(bmap);
                ord = bd.getLightOrder();
                if(ord == BeaconFinder.LightOrder.BLUE_RED ||
                   ord == BeaconFinder.LightOrder.RED_BLUE)
                {
                    break;
                }
                else
                {
                    sleep(50);
                }
                frame++;
            }
        }
        tracker.setActive(false);
        tracker.setFrameQueueSize(0);

        if (ord != BeaconFinder.LightOrder.UNKNOWN)
        {
            double t = itimer.seconds();
            RobotLog.ii("SJH", "Found Beacon!!! %s %3.3f frame: %d", ord, t, frame);
            telemetry.addData("BORD", "SJH LightOrder = %s frame %d", ord, frame);
        }
        telemetry.update();
    }

    @Override
    public void runOpMode()
    {
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Starting ...");
        telemetry.update();

        //tracker.setFrameQueueSize(10);

        telemetry.addData(":", "Visual Cortex activated!");
        RobotLog.ii("SJH", "Visual Cortex activated!");
        telemetry.update();

        /** Start tracking */
        timer.reset();

        while (opModeIsActive())
        {
            boolean locFound = findSensedLoc();
            do_findBeaconOrder(false);
            sleep(2000);
            idle();
        }
        tracker.setActive(false);
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
