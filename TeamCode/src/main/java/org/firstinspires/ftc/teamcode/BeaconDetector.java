package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class BeaconDetector {
    private final static double MIN_COLOR_ZONE_AREA = 0.05; 	// fraction of total image area
    private final static double MIN_BUTTON_AREA = 0.01; 		// fraction of total image area
    private final static double MIN_BUTTON_EDGE_DIST = 20; 		// pixels from edge of cropped image

    private enum FindMode {LIGHT, BUTTON}

    private Mat image;

    private Point blue_button;
    private Point red_button;


    @SuppressWarnings("WeakerAccess")
    public BeaconDetector(Mat img ) {
        setImage( img );
    }

    public void setImage( Mat img )
    {
        // Assert: image is a CvMat object
        // Convert to HSV colorspace to make it easier to
        // threshold certain colors (ig red/blue)
        image = new Mat();
        Imgproc.cvtColor(img, image, Imgproc.COLOR_RGB2HSV );

        findColors();
    }

    // Is there a blue region in the image
    public boolean isBlue()
    {
        return blue_button !=null;
    }

    // Point of the center of the blue button
    @SuppressWarnings("WeakerAccess")
    public Point blueButton()
    {
        return blue_button;
    }

    // Is there a red region in the image
    public boolean isRed()
    {
        return red_button !=null;
    }

    // Point of the center of the red button
    @SuppressWarnings("WeakerAccess")
    public Point redButton ()
    {
        return red_button;
    }

    private void findColors()
    {
        findBlue();
        findRed();
    }

    private void findBlue()
    {
        Mat blue_areas = new Mat();
        // Threshold based on color.  White regions match the desired color.  Black do not.
        // We now have a binary image to work with.  Contour detection looks for white blobs
        Core.inRange( image, new Scalar( 110,100,100 ), new Scalar( 130,255,255 ), blue_areas );

        // There can be several blobs.  Find the largest that fills a certain amount
        // of the image.  These are crude heuristics but should be fine if we control
        // the conditions of when we start searching (ie, appx size of beacon in image
        // frame, etc).
        Rect blue_light_box = findLargestObject(blue_areas, FindMode.LIGHT);
        if ( blue_light_box == null ) return;

        // Now we want to find the button.  It should be in the blue area
        // we just found so, crop out that area and search for it.
        Mat blue_crop = blue_areas.submat(blue_light_box);

        // The button is a black blob inside a white area.  So we need to invert
        // the image to get a white blob inside a black area.  The problem we'll
        // run into is there will be black regions around the edge (because the beacon
        // is not square) and we don't want those to be considered.
        Rect blue_button_box = findLargestObject(invert(blue_crop), FindMode.BUTTON);
        if ( blue_button_box == null ) return;

        blue_button = new Point( blue_button_box.x, blue_button_box.y );
    }

    private void findRed()
    {
        // Same game, just a different hue
        Mat red_areas = new Mat();
        Core.inRange( image, new Scalar( 0,100,100 ), new Scalar( 10,255,255 ), red_areas);

        Rect red_light_box = findLargestObject(red_areas, FindMode.LIGHT);
        if ( red_light_box == null ) DbgLog.msg("SH BOOO :(");
        if ( red_light_box == null ) return;
        DbgLog.msg( "SH SZ: " + String.valueOf( red_light_box.width ) + "x" + String.valueOf( red_light_box.height ) );
        Mat red_crop = red_areas.submat(red_light_box);
        Rect red_button_box = findLargestObject(invert(red_crop), FindMode.BUTTON);
        if ( red_button_box == null ) return;

        red_button = new Point( red_button_box.x, red_button_box.y );
    }

    private Mat invert( Mat img )
    {
        Mat inv = new Mat();
        Mat white = img.clone();
        white = white.setTo( new Scalar( 255 ) );
        Core.subtract( white, img, inv );
        return inv;
    }

    private Rect findLargestObject( Mat img, FindMode mode )
    {

        double barea = 0;
        double carea;
        Rect bbox;
        MatOfPoint bigr = null;
        double h = img.height();
        double w = img.width();
        double ima = h * w;
        boolean cdn;

        Mat hchy = new Mat();
        List<MatOfPoint> ctr = new ArrayList<>();

        Imgproc.findContours( img, ctr, hchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE );

        // Loop over each blob (contour)
        for (MatOfPoint wrapper : ctr)
        {
            bbox = Imgproc.boundingRect(wrapper);
            carea = bbox.width * bbox.height;

            if (mode == FindMode.BUTTON)
            {
                // Only blobs not along edge
                cdn = bbox.x > MIN_BUTTON_EDGE_DIST &&
                      bbox.y > MIN_BUTTON_EDGE_DIST &&
                      bbox.x + bbox.width < w - MIN_BUTTON_EDGE_DIST &&
                      bbox.y + bbox.height < h - MIN_BUTTON_EDGE_DIST &&
                      Imgproc.contourArea(wrapper) / ima > MIN_BUTTON_AREA;
            }
            else // mode == FindMode.LIGHT
            {
                // Only blobs that make up a sizable area of the image

                cdn = Imgproc.contourArea(wrapper) / ima > MIN_COLOR_ZONE_AREA;
                DbgLog.msg("SH BOX: " + String.valueOf(Imgproc.contourArea(wrapper)) + ":" + String.valueOf(ima));
            }

            if (cdn && (bigr == null || carea > barea))
            {
                bigr = wrapper;
                barea = carea;
            }
        }

        if ( bigr == null ) return null;
        DbgLog.msg("SH WEEEE!!!!!: ");
        return Imgproc.boundingRect(bigr);
    }

    public static void main(String args[])
    {
        //TODO: get image
        Mat mat = new Mat();
        BeaconDetector bd = new BeaconDetector(mat);
        Point rb = null;
        if (bd.isRed())  rb = bd.redButton();
        Point bb = null;
        if (bd.isBlue()) bb = bd.blueButton();
        if(rb != null) DbgLog.msg("BluePoint %s", rb.toString());
        if(bb != null) DbgLog.msg("BluePoint %s", bb.toString());
    }
}
