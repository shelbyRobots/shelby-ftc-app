package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.ftccommon.DbgLog;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class BeaconDetector implements BeaconFinder
{
    private final static double MIN_COLOR_ZONE_AREA = 0.05; 	// fraction of total image area
    private final static double MIN_BUTTON_AREA = 0.01; 		// fraction of total image area
    private final static double MIN_BUTTON_EDGE_DIST = 20; 		// pixels from edge of cropped image

    private Mat image;

    private double blue_light_box = -1;
    private double red_light_box = -1;
    private LightOrder light_order;

    private final static boolean DEBUG = true;
    private final static boolean POS_IS_Y = false;

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
        Imgproc.cvtColor(img, image, Imgproc.COLOR_RGB2HSV, 4 );

        findColors();
    }

    public LightOrder getLightOrder() {
        return light_order;
    }

    public void calcLightOrder() {

        light_order = LightOrder.UNKNOWN;
        if(DEBUG)
        {
            DbgLog.msg("SJH: r: " + String.valueOf(red_light_box) +
                               " b: " + String.valueOf(blue_light_box));
        }

        if ( blue_light_box != -1 && red_light_box != -1 ) {
            if ( blue_light_box < red_light_box )
                light_order = LightOrder.BLUE_RED;
            else
                light_order = LightOrder.RED_BLUE;
        } else if ( blue_light_box != -1 ) {
            light_order = LightOrder.BLUE_BLUE;
        } else if ( red_light_box != -1 ) {
            light_order = LightOrder.RED_RED;
        }

    }

    private void findColors()
    {
        findBlue();
        findRed();
        calcLightOrder();
        //findAverage();
    }

    private void findBlue()
    {
        Mat blue_areas = new Mat();

        blue_light_box = -1;

        // Threshold based on color.  White regions match the desired color.  Black do not.
        // We now have a binary image to work with.  Contour detection looks for white blobs
        Core.inRange( image, new Scalar( 105,100,100 ), new Scalar( 125,255,255 ), blue_areas );

        // There can be several blobs.  Find the largest that fills a certain amount
        // of the image.  These are crude heuristics but should be fine if we control
        // the conditions of when we start searching (ie, appx size of beacon in image
        // frame, etc).
        if(DEBUG) DbgLog.msg("SJH: BLUE");
        blue_light_box = findWeightedPos(blue_areas);
    }

    private void findRed()
    {
        // Same game, just a different hue
        Mat red1 = new Mat();
        Mat red2 = new Mat();
        Mat red_areas = new Mat();

        red_light_box = -1;

        Core.inRange( image, new Scalar( 0,100,100 ), new Scalar( 10,255,255 ), red1);
        Core.inRange( image, new Scalar( 160,100,100 ), new Scalar( 179,255,255 ), red2);
        Core.bitwise_or(red1, red2, red_areas);

        if(DEBUG) DbgLog.msg("SJH: RED");
        red_light_box = findWeightedPos( red_areas );
    }

    private Rect findLargestObject( Mat img )
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

            cdn = Imgproc.contourArea(wrapper) / ima > MIN_COLOR_ZONE_AREA;

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

    private double findWeightedPos( Mat img )
    {

        Rect bbox;
        double barea;

        double biggest = 0;

        double h = img.height();
        double w = img.width();
        double ima = h * w;

        double xmid, xwgt, xsum = 0, asum = 0;
        double ymid, ywgt, ysum = 0;

        Mat hchy = new Mat();
        List<MatOfPoint> ctr = new ArrayList<>();

        Imgproc.findContours( img, ctr, hchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE );

        // Loop over each blob (contour)
        for (MatOfPoint wrapper : ctr)
        {
            bbox = Imgproc.boundingRect(wrapper);
            barea = Imgproc.contourArea(wrapper);

            xmid = bbox.width / 2 + bbox.x;
            ymid = bbox.height / 2 + bbox.y;
            xwgt = barea * xmid;
            ywgt = barea *ymid;


            xsum += xwgt;
            ysum += ywgt;
            asum += barea;

            if (barea > biggest) biggest = barea;
        }

        //DbgLog.msg("R: "+ String.valueOf(xsum)+ "|"+ String.valueOf(asum));
        // return -1 if sum of area is less
        // than MIN_COLOR_ZONE_AREA of total image
        // area
        if(asum / ima < MIN_COLOR_ZONE_AREA) return -1;
        // otherwise, return average sum of weighted x midpoints

        if(DEBUG)
        {
            DbgLog.msg("SJH: CNT %d BIG %6.1f / %d X %3.1f Y %3.1f %d x %d",
                    ctr.size(), biggest, image.width() * image.height(),
                    xsum / asum, ysum / asum,
                    image.width(), image.height());
        }

        double retValue = xsum / asum;
        if(POS_IS_Y) retValue = ysum / asum;
        return retValue;
    }

    private void findAverage()
    {
        Bitmap bitmap = null;
        try {
            bitmap = Bitmap.createBitmap(image.cols(), image.rows(),
                                         Bitmap.Config.ARGB_8888);
        }
        catch (Exception e){DbgLog.error("houston we have a problem");}

        int hueBucket = 0;
        int satBucket = 0;
        int valBucket = 0;

        if(bitmap == null) return;

        Utils.matToBitmap(image, bitmap);
        int pixelCount = bitmap.getWidth() * bitmap.getHeight();
        int[] pixels = new int[pixelCount];
        bitmap.getPixels(pixels, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());

        for (int y = 0, h = bitmap.getHeight(); y < h; y++)
        {
            for (int x = 0, w = bitmap.getWidth(); x < w; x++)
            {
                int color = pixels[x + y * w];
                hueBucket += (color >> 16) & 0xFF;
                satBucket += (color >> 8) & 0xFF;
                valBucket += color & 0xFF;
            }
        }

        DbgLog.msg( " H: " + String.valueOf( hueBucket / pixelCount ) +
                " S: " + String.valueOf( satBucket / pixelCount ) +
                " V: " + String.valueOf( valBucket / pixelCount ) );
    }
}
