package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.Range;

import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;


public class BeaconDetector implements BeaconFinder
{
    private final static double MIN_COLOR_ZONE_AREA = 0.2; 	    // fraction of total image area

    private Mat image;
    private Mat zonedImg;
    private Mat maskImg;
    private Mat showImg;
    private Mat cvImage;

    private double blue_light_box = -1;
    private double red_light_box = -1;
    private LightOrder light_order;

    private final static boolean DEBUG = false;
    private final static boolean POS_IS_Y = false;

    private List<MatOfPoint> red_blobs = new ArrayList<MatOfPoint>();
    private List<MatOfPoint> blue_blobs = new ArrayList<MatOfPoint>();
    private List<MatOfPoint> white_blobs = new ArrayList<MatOfPoint>();
    private List<MatOfPoint> black_blobs = new ArrayList<MatOfPoint>();

    private Rect red_box;
    private Rect blue_box;
    private Rect white_box;
    private Rect beacon_box;
    private List<Rect> buttons = new ArrayList<Rect>();

    private RingBuffer beaconConfBuf = new RingBuffer(20);
    private RingBuffer beaconPosXBuf = new RingBuffer(15);
    private RingBuffer beaconPosZBuf = new RingBuffer(2);

    private double beaconConf = 0;
    private double beaconPosX = 0;
    private double beaconPosZ = 0;

    private BeaconSide redPosSide = BeaconSide.UNKNOWN;
    private BeaconSide bluePosSide = BeaconSide.UNKNOWN;

    private boolean sensingActive = false;
    private boolean firstCalcsDone = false;

    enum BeaconSide
    {
        UNKNOWN,
        LEFT,
        RIGHT
    }

    enum Color
    {
        WHITE,
        BLUE,
        RED,
        BLACK
    }

    static
    {
        if (!OpenCVLoader.initDebug()) {
            DbgLog.error("SJH: OpenCVLoader error"); //Handle opencv loader issue
        }
    }

    @SuppressWarnings("WeakerAccess")
    public BeaconDetector(Mat img ) {
        setImage( img );
    }

    public BeaconDetector() {}

    public void startSensing() {
        beaconConfBuf.clear();
        beaconPosXBuf.clear();
        beaconPosZBuf.clear();

        firstCalcsDone = false;
        sensingActive = true;
    }

    public void stopSensing() {
        firstCalcsDone = false;
        sensingActive = false;
    }

    public void setImage( Mat img )
    {
        // Convert to HSV colorspace to make it easier to
        // threshold certain colors (ig red/blue)
        image = new Mat();

        Imgproc.cvtColor( img, image, Imgproc.COLOR_RGB2HSV, 4 );
        showImg = image.clone();

        if ( !sensingActive ) return;
        findColors();
        firstCalcsDone = true;
    }

    public LightOrder getLightOrder() {
        return light_order;
    }

    public void calcLightOrder() {

        light_order = LightOrder.UNKNOWN;

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

    public void logDebug()
    {
        DbgLog.msg("SJH: CONF: %5.2f, X: %5.2f, Z: %5.2f, RED: %s, BLUE: %s",
                beaconConf,
                beaconPosX,
                beaconPosZ,
                redPosSide,
                bluePosSide);
    }

    public synchronized BeaconSide getRedPosSide() { return redPosSide; }
    public synchronized BeaconSide getBluePosSide() { return bluePosSide; }
    public synchronized double getBeaconConf() { return beaconConf; }
    public synchronized double getBeaconPosX() { return beaconPosX; }
    public synchronized double getBeaconPosZ() { return beaconPosZ; }

    private void calcPosition()
    {
        beaconConf = 0;
        redPosSide = BeaconSide.UNKNOWN;
        bluePosSide = BeaconSide.UNKNOWN;

        if ( beacon_box.height == 0 || beacon_box.width == 0) return;

        double scrn_ctr = image.cols() / 2;
        double beac_ctr = beacon_box.x + beacon_box.width / 2;

        double beac_w = 8.5;
        double beac_h = 6.5;
        double butn_d = 5.25;

        double actl_beac_rt = beac_w / beac_h;
        double actl_butn_rt = butn_d / beac_w;

        double sens_bcn_rt = beacon_box.width / beacon_box.height;
        double sens_btn_rt = 0.0;

        if ( buttons.size() == 2 )
            sens_btn_rt = Math.abs( buttons.get( 0 ).x - buttons.get( 1 ).x ) / beacon_box.width;

        double beac_rt = beacon_box.area() / ( image.cols() * image.rows() );
        double red_rt = red_box.area() / beacon_box.area();
        double blue_rt = blue_box.area() / beacon_box.area();

//        DbgLog.msg( "SJH BCN: %5.2f, BTN: %5.2f, BCN2: %5.2f, RED: %5.2f, BLUE: %5.2f",
//                    Range.clip( sens_bcn_rt - actl_beac_rt, -1.0, 1.0 ) * 1.0 / actl_beac_rt,
//                    Range.clip( sens_btn_rt - actl_butn_rt, -1.0, 1.0 ) * 1.0 / actl_butn_rt,
//                    Range.clip( 0.6 - beac_rt, 0, 0.6 ) * 1.67,
//                    ( 0.4 - red_rt ) * 2.5,
//                    ( 0.4 - blue_rt ) * 2.5
//                );


        beaconConf = beaconConfBuf.smooth(
                        Range.clip( 1 - Math.sqrt(
                          ( Math.pow( Range.clip( sens_bcn_rt - actl_beac_rt, -1.0, 1.0 ) / actl_beac_rt, 2 ) +
                            Math.pow( Range.clip( sens_btn_rt - actl_butn_rt, -1.0, 1.0 ) / actl_butn_rt, 2 ) +
                            3 * Math.pow( Range.clip( 0.6 - beac_rt, -0.6, 0.6 ) * 1.67, 2 ) +
                            2 * Math.pow( Range.clip( 0.4 - ( red_rt + blue_rt ) / 2, -0.4, 0.4 ) * 2.5, 2 )
                          ) / 7.0 ), 0.0, 1.0 )
                    );

        beaconPosX = beaconPosXBuf.smooth( beac_ctr - scrn_ctr );
        beaconPosZ = beaconPosZBuf.smooth( (double) beacon_box.width / (double) image.cols() );

        // Keep in mind that we flip the image before
        // processing to make it easier to visualize
        if ( red_box.width < 5 || red_box.height < 5 )
            redPosSide = BeaconSide.UNKNOWN;
        else if ( beac_ctr > red_box.x + red_box.width / 2 )
            redPosSide = BeaconSide.RIGHT;
        else
            redPosSide = BeaconSide.LEFT;

        if ( blue_box.width < 5 || blue_box.height < 5 )
            bluePosSide = BeaconSide.UNKNOWN;
        else if ( beac_ctr > blue_box.x + blue_box.width / 2 )
            bluePosSide = BeaconSide.RIGHT;
        else
            bluePosSide = BeaconSide.LEFT;

        if ( redPosSide == bluePosSide || beaconConf < 0.3 )
        {
            redPosSide = BeaconSide.UNKNOWN;
            bluePosSide = BeaconSide.UNKNOWN;
        }

    }

    private double minOf( double w, double r, double b )
    {
        return ( w + 2 * Math.min( r, b ) ) / 3;
    }

    private double maxOf( double w, double r, double b )
    {
        return ( w + 2 * Math.max( r, b ) ) / 3;
    }

    private void findBeaconBox()
    {
        double tx = minOf(white_box.x, red_box.x, blue_box.x);
        double bx = maxOf(white_box.br().x, red_box.br().x, blue_box.br().x);
        double ty = minOf(white_box.y, red_box.y, blue_box.y);
        double by = maxOf(white_box.br().y, red_box.br().y, blue_box.br().y);

        beacon_box = new Rect( (int) tx, (int) ty, (int)(bx - tx), (int)(by - ty) );
    }

    private void findColors()
    {
        findLum();
        findBlue();
        findRed();
        findBeaconBox();
        findButtons();
        calcPosition();

        calcLightOrder();
    }

    private void findLum()
    {
        List<Mat> channels = new ArrayList<Mat>();
        Core.split( image, channels );

        Mat sat = channels.get( 1 );
        Mat lum = channels.get( 2 );

        double lumAvg = Core.mean( lum ).val[0];

        Core.normalize( sat.clone(), sat, 120, 255, Core.NORM_MINMAX );
        Core.normalize( lum.clone(), lum, 0, 180, Core.NORM_MINMAX );

        Mat white = lum.clone();
        Mat adj = lum.clone();

        Imgproc.GaussianBlur( lum, adj, new Size(25,25), 25);
        Imgproc.threshold( adj, white, 255 - lumAvg, 255, Imgproc.THRESH_BINARY ); //+ Imgproc.THRESH_OTSU
        Imgproc.erode( white.clone(), white, Imgproc.getGaussianKernel( 5, 2 ) );

        findWeightedPos( white, Color.WHITE );

        channels.set( 1, sat );
        channels.set( 2, lum );

        zonedImg = image.clone();
        Core.merge( channels, zonedImg );

        List<Mat> tmp = new ArrayList<Mat>();
        Core.split( image, tmp );
        Mat mask = image.clone();

        Imgproc.dilate( white.clone(), white, Imgproc.getGaussianKernel( 5, 2 ) );
        tmp.set( 0, Mat.ones( image.rows(), image.cols(), white.type() ) );
        tmp.set( 1, Mat.ones( image.rows(), image.cols(), white.type() ) );
        tmp.set( 2, white );
        Core.merge( tmp, mask );

        Core.multiply( zonedImg.clone(), mask, zonedImg );
//        showImg = zonedImg.clone();
    }

    private void findButtons()
    {
        buttons.clear();
        black_blobs.clear();

        List<Mat> channels = new ArrayList<Mat>();
        Core.split( zonedImg, channels );

        Mat s_value = channels.get( 2 );
        Mat d_value = s_value.clone();
        //Mat inv = new Mat( d_value.rows(), d_value.cols(), d_value.type(), new Scalar( 255 ) );
        //Imgproc.adaptiveThreshold( d_value.clone(), d_value, 255.0, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, 15, 0 );
        //Core.subtract( inv, d_value.clone(), d_value );
        //Imgproc.GaussianBlur( d_value.clone(), d_value, new Size(15,15), 15);
        Imgproc.threshold( d_value.clone(), d_value, 40, 255, Imgproc.THRESH_BINARY_INV ); // + Imgproc.THRESH_OTSU );

//        channels.set( 0, Mat.zeros( image.rows(), image.cols(), d_value.type() ) );
//        channels.set( 1, Mat.zeros( image.rows(), image.cols(), d_value.type() ) );
//        channels.set( 2, d_value );
//
//        showImg = zonedImg.clone();
//        Core.merge( channels, showImg );

        Mat hchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        Imgproc.findContours(d_value, contours, hchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        Rect bounded_box;
        double maxWidth = (double) beacon_box.width * 0.2;
        double minWidth = (double) beacon_box.width * 0.015;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            bounded_box = Imgproc.boundingRect(wrapper);
            // Reasonably sized
            if (bounded_box.width > minWidth &&
                    bounded_box.width < maxWidth &&
                    // Inside beacon box
                    bounded_box.x  > beacon_box.x &&
                    bounded_box.x + bounded_box.width < beacon_box.x + beacon_box.width &&
                    bounded_box.y > beacon_box.y &&
                    bounded_box.y + bounded_box.height < beacon_box.y + beacon_box.height ) {

                    black_blobs.add( wrapper );
                    buttons.add( bounded_box );

            }
        }
    }

    private void findBlue()
    {
        Mat blue_areas = new Mat();

        blue_light_box = -1;

        // Threshold based on color.  White regions match the desired color.  Black do not.
        // We now have a binary image to work with.  Contour detection looks for white blobs
        Core.inRange( zonedImg, new Scalar( 105,100,100 ), new Scalar( 125,255,255 ), blue_areas );
        Imgproc.dilate( blue_areas.clone(), blue_areas, new Mat() );

        // There can be several blobs.  Find the largest that fills a certain amount
        // of the image.  These are crude heuristics but should be fine if we control
        // the conditions of when we start searching (ie, appx size of beacon in image
        // frame, etc).
        blue_light_box = findWeightedPos(blue_areas, Color.BLUE);
    }

    private void findRed()
    {
        // Same game, just a different hue
        Mat red1 = new Mat();
        Mat red2 = new Mat();
        Mat red_areas = new Mat();

        red_light_box = -1;

        Core.inRange( zonedImg, new Scalar( 0,100,150 ), new Scalar( 10,255,255 ), red1);
        Core.inRange( zonedImg, new Scalar( 140,100,150 ), new Scalar( 179,255,255 ), red2);
        Core.bitwise_or(red1, red2, red_areas);
        Imgproc.dilate( red_areas.clone(), red_areas, new Mat() );

        red_light_box = findWeightedPos( red_areas, Color.RED );
    }

    private double findWeightedPos( Mat img, Color find )
    {

        Rect bbox = findColorBlobs( img, find );

        switch (find) {
            case BLUE:
                blue_box = bbox;
                break;
            case RED:
                red_box = bbox;
                break;
            case WHITE:
                white_box = bbox;
                break;
        }

        return bbox.x + bbox.width / 2 ;
    }

    public Rect findColorBlobs( Mat img, Color find ) {

        Mat hchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        List<MatOfPoint> calcCtr;
        calcCtr = null;

        switch (find) {
            case BLUE:
                calcCtr = blue_blobs;
                break;
            case RED:
                calcCtr = red_blobs;
                break;
            case WHITE:
                calcCtr = white_blobs;
                break;
        }

        Imgproc.findContours(img, contours, hchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        double maxArea = 0;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea)
                maxArea = area;
        }

        // Filter contours by area and resize to fit the original image size

        double contour_area;
        double a_sum = 0, w_x = 0, w_h = 0, w_y = 0, w_w = 0;
        Rect bounded_box;

        calcCtr.clear();
        each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint contour = each.next();
            contour_area = Imgproc.contourArea( contour );
            if ( contour_area > MIN_COLOR_ZONE_AREA * maxArea ) {
                calcCtr.add(contour);

                bounded_box = Imgproc.boundingRect( contour );

                a_sum += contour_area;
                w_x += contour_area * bounded_box.x;
                w_y += contour_area * bounded_box.y;
                w_h += contour_area * bounded_box.height;
                w_w += contour_area * bounded_box.width;
            }
        }

        return new Rect(
                (int)(w_x / a_sum),
                (int)(w_y / a_sum),
                (int)(w_w / a_sum),
                (int)(w_h / a_sum)
                );
    }

    public Mat drawBeacon() {

        Mat out = image.clone();
        Imgproc.cvtColor( showImg, out, Imgproc.COLOR_HSV2RGB, 4 );
        if ( !sensingActive || !firstCalcsDone ) return out;

        Imgproc.rectangle( out, beacon_box.tl(), beacon_box.br(), new Scalar(200,200,200), -1 );
        Imgproc.rectangle( out, blue_box.tl(), blue_box.br(), new Scalar(50,50,255), -1 );
        Imgproc.rectangle( out, red_box.tl(), red_box.br(), new Scalar(255,50,50), -1 );

        for ( Rect butn : buttons )
        {
            Imgproc.rectangle( out, butn.tl(), butn.br(), new Scalar(40,40,40), -1 );
        }

        Imgproc.drawContours( out, blue_blobs, -1, new Scalar(0,0,255), 2 );
        Imgproc.drawContours( out, red_blobs, -1, new Scalar(255,0,0), 2 );
        Imgproc.drawContours( out, white_blobs, -1, new Scalar(255,255,255), 2 );
        Imgproc.drawContours( out, black_blobs, -1, new Scalar(0,0,0), 2 );

        return out;
    }

    public void setBitmap(Bitmap rgbImage)
    {
        if(rgbImage == null) return;

        int cvt = CvType.CV_8UC1;
        int inHeight = rgbImage.getHeight();
        int inWidth  = rgbImage.getWidth();

        if (cvImage == null) cvImage = new Mat(inHeight, inWidth, cvt);
        if (image == null)     image = new Mat(inHeight, inWidth, cvt);

        Utils.bitmapToMat(rgbImage, cvImage);
        Imgproc.cvtColor(cvImage, image, Imgproc.COLOR_RGB2HSV, 4 );

        findColors();
    }
}
