package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class BeaconDetector implements BeaconFinder {
    private final static double MIN_COLOR_ZONE_AREA = 0.05; 	// fraction of total image area
    private final static double MIN_BUTTON_AREA = 0.01; 		// fraction of total image area
    private final static double MIN_BUTTON_EDGE_DIST = 20; 		// pixels from edge of cropped image

    private enum FindMode {LIGHT, BUTTON}

    private Mat image;

    private Rect blue_light_box;
    private Rect red_light_box;


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

    public LightOrder getLightOrder(){
        LightOrder f = LightOrder.UNKNOWN;

        if ( blue_light_box != null && red_light_box != null ) {
            if ( blue_light_box.x > red_light_box.x )
                f = LightOrder.BLUE_RED;
            else
                f = LightOrder.RED_BLUE;
        } else if ( blue_light_box != null ) {
            f = LightOrder.BLUE_BLUE;
        } else if ( red_light_box != null ) {
            f = LightOrder.RED_RED;
        }

        return f;
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
        blue_light_box = findLargestObject(blue_areas, FindMode.LIGHT);
    }

    private void findRed()
    {
        // Same game, just a different hue
        Mat red_areas = new Mat();

        Core.inRange( image, new Scalar( 0,100,100 ), new Scalar( 10,255,255 ), red_areas);
        red_light_box = findLargestObject(red_areas, FindMode.LIGHT);

        if ( red_light_box == null ) {
            Core.inRange(image, new Scalar(160, 100, 100), new Scalar(179, 255, 255), red_areas);
            red_light_box = findLargestObject(red_areas, FindMode.LIGHT);
        }

    }

    private Rect findLargestObject( Mat img, FindMode mode )
    {

        double barea = 0;
        double carea;
        Rect bbox;
        Rect fbox = null;
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

                DbgLog.msg("SH X,Y: " + String.valueOf(bbox.x) + "," + String.valueOf(bbox.y));
                DbgLog.msg("SH W,H: " + String.valueOf(bbox.width) + "," + String.valueOf(bbox.height));
            }
            else // mode == FindMode.LIGHT
            {
                // Only blobs that make up a sizable area of the image

                cdn = Imgproc.contourArea(wrapper) / ima > MIN_COLOR_ZONE_AREA;
            }

            if (cdn && (bigr == null || carea > barea))
            {
                bigr = wrapper;
                barea = carea;
            }
        }

        if ( bigr != null )
           fbox = Imgproc.boundingRect(bigr);

        return fbox;
    }
}
