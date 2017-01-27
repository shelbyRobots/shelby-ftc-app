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
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;


public class BeaconDetector implements BeaconFinder
{
    private final static double MIN_COLOR_ZONE_AREA = 0.2;// fraction of total image area
    private final static double IMAGE_WIDTH = 864.0;
    private final static double IMAGE_SCALE_FACTOR = IMAGE_WIDTH / 864.0;


    private Mat image;
    private Mat zonedImg;
    private Mat maskImg;
    private Mat showImg;
    private Mat cvImage;
    private Mat colorDiff;

    private final static boolean DEBUG = false;
    private final static boolean POS_IS_Y = false;

    private List<MatOfPoint> red_blobs = new ArrayList<>();
    private List<MatOfPoint> blue_blobs = new ArrayList<>();
    private List<MatOfPoint> white_blobs = new ArrayList<>();
    private List<MatOfPoint> black_blobs = new ArrayList<>();

    private ArrayList<Rect> red_matches = new ArrayList<>();
    private ArrayList<Rect> blue_matches = new ArrayList<>();
    private ArrayList<Rect> white_matches = new ArrayList<>();

    private Rect red_box;
    private Rect blue_box;
    private Rect white_box;
    private Rect beacon_box;
    private List<Rect> buttons = new ArrayList<>();

    private RingBuffer beaconConfBuf = new RingBuffer(20);
    private RingBuffer beaconPosXBuf = new RingBuffer(3);
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
        colorDiff = new Mat();

        Imgproc.cvtColor( img, image, Imgproc.COLOR_RGB2HSV, 4 );
        showImg = image.clone();

        List<Mat> channels = new ArrayList<>();
        Core.split( img, channels );
        Mat red = channels.get( 0 );
        Mat blue = channels.get( 2 );
        Core.absdiff( red, blue, colorDiff );
        Imgproc.threshold( colorDiff.clone(), colorDiff, 20, 255, Imgproc.THRESH_BINARY );

        if ( !sensingActive ) return;
        findColors();
        firstCalcsDone = true;
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

    private double scoreFit( Rect wb, Rect rb, Rect bb )
    {
        double beac_w = 9.0;
        double beac_h = 6.5;

        double actl_beac_rt = beac_w / beac_h;
        double sens_bcn_rt = wb.width / wb.height;

        double beac_rt = wb.area() / ( image.cols() * image.rows() );
        double red_rt = rb.area() / wb.area();
        double blue_rt = bb.area() / wb.area();

        double beac_aspect_factor =
                Math.pow( Range.clip( sens_bcn_rt - actl_beac_rt, -1.0, 1.0 ) / actl_beac_rt, 2 );
        double wb_ratio_factor =
                Math.pow( Range.clip( 0.6 - beac_rt, -0.6, 0.6 ) * 1.67, 2 );
        double rb_ratio_factor =
                Math.pow( Range.clip( 0.4 - ( red_rt + blue_rt ) / 2, -0.4, 0.4 ) * 2.5, 2 );

        DbgLog.msg("SJH: scoreFit beac_apsect_factor %4.3f" +
                   "wb_ratio_factor %4.3f rb_ratio_factor %4.3f",
                beac_aspect_factor, wb_ratio_factor, rb_ratio_factor);

        return Range.clip( 1 - Math.sqrt(
                    ( beac_aspect_factor +
                      3 * wb_ratio_factor +
                      2 * rb_ratio_factor
                    ) / 6.0 ), 0.0, 1.0 );
    }

    private void calcPosition()
    {

        if ( beacon_box.height == 0 || beacon_box.width == 0)
        {
            beaconConf = beaconConfBuf.smooth( 0.0 );

            if ( redPosSide == bluePosSide || beaconConf < 0.3 )
            {
                redPosSide = BeaconSide.UNKNOWN;
                bluePosSide = BeaconSide.UNKNOWN;
            }

            return;
        }

        double scrn_ctr = image.cols() / 2;
        double beac_ctr = beacon_box.x + beacon_box.width / 2;

        beaconConf = beaconConfBuf.smooth( scoreFit( beacon_box, red_box, blue_box ) );
        beaconPosZ = beaconPosZBuf.smooth( ((double) beacon_box.width * -0.0407 + 34.0829) / IMAGE_SCALE_FACTOR );
        beaconPosX = beaconPosXBuf.smooth( (beac_ctr - scrn_ctr) / (864.0 / ( 1.3 * beaconPosZ - 2.4)));

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

    private Rect bestFit( Rect wb, ArrayList<Rect> matches )
    {
        int w_ctr_x = wb.x + wb.width / 2;
        Double mp1, mp2, mp3, mpf;
        Double minFit = Double.POSITIVE_INFINITY;
        Rect best = new Rect( wb.x, wb.y, 1, 1 );

        for ( Rect cb : matches )
        {
            if ( wb.contains( new Point( cb.x + cb.width / 2, cb.y + cb.height / 2 ) ) )
            {
                mp1 = Math.pow( wb.tl().y - cb.tl().y, 2 );
                mp2 = Math.pow( wb.br().y - cb.br().y, 2 );
                mp3 = Math.min( Math.pow( w_ctr_x - cb.tl().x, 2 ), Math.pow( w_ctr_x - cb.br().x, 2 ) );

                mpf = Math.sqrt( mp1 + mp2 + mp3 / 3.0 );
                if ( mpf < minFit )
                {
                    minFit = mpf;
                    best = cb;
                }
            }
        }

        return best;
    }

    private void findBeaconBox()
    {
        if ( white_matches.size() == 0 )
        {
            white_box = new Rect( 0, 0, 1, 1 );
            blue_box = new Rect( 0, 0, 1, 1 );
            red_box = new Rect( 0, 0, 1, 1 );
            beacon_box = new Rect( 0, 0, 1, 1 );
        }
        else
        {
            double fs, maxFit = 0.0;
            Rect rb, bb;

            for ( Rect wb : white_matches )
            {
                rb = bestFit( wb, red_matches );
                bb = bestFit( wb, blue_matches );

                fs = scoreFit( wb, rb, bb );
                if ( fs > maxFit )
                {
                    maxFit = fs;

                    white_box = wb;
                    red_box = rb;
                    blue_box = bb;
                }
            }

            double tx = minOf(white_box.x, red_box.x, blue_box.x);
            double bx = maxOf(white_box.br().x, red_box.br().x, blue_box.br().x);
            double ty = minOf(white_box.y, red_box.y, blue_box.y);
            double by = maxOf(white_box.br().y, red_box.br().y, blue_box.br().y);

            beacon_box = new Rect( (int) tx, (int) ty, (int)(bx - tx), (int)(by - ty) );
        }

    }

    private void findColors()
    {
        findLum();
        findBlue();
        findRed();
        findBeaconBox();
        findButtons();
        calcPosition();
    }

    private void setDebugImg( Mat ch )
    {
        List<Mat> channels = new ArrayList<>();
        Core.split( image, channels );

        channels.set( 0, Mat.zeros( image.rows(), image.cols(), ch.type() ) );
        channels.set( 1, Mat.zeros( image.rows(), image.cols(), ch.type() ) );
        channels.set( 2, ch );

        showImg = image.clone();
        Core.merge( channels, showImg );
    }

    private void findLum()
    {
        List<Mat> channels = new ArrayList<>();
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

//        setDebugImg( white );

        findWeightedPos( white, white_blobs, white_matches );

        channels.set( 1, sat );
        channels.set( 2, lum );

        zonedImg = image.clone();
        Core.merge( channels, zonedImg );

        List<Mat> tmp = new ArrayList<>();
        Core.split( image, tmp );
        Mat mask = image.clone();

        Imgproc.dilate( white.clone(), white, Imgproc.getGaussianKernel( 5, 2 ) );
        tmp.set( 0, Mat.ones( image.rows(), image.cols(), white.type() ) );
        tmp.set( 1, Mat.ones( image.rows(), image.cols(), white.type() ) );
        tmp.set( 2, white );
        Core.merge( tmp, mask );

        Core.multiply( zonedImg.clone(), mask, zonedImg );
        //showImg = zonedImg.clone();
    }

    private void findButtons()
    {
        buttons.clear();
        black_blobs.clear();

        List<Mat> channels = new ArrayList<>();
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
        List<MatOfPoint> contours = new ArrayList<>();

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

        // Threshold based on color.  White regions match the desired color.  Black do not.
        // We now have a binary image to work with.  Contour detection looks for white blobs
        Core.inRange( zonedImg, new Scalar( 105,100,100 ), new Scalar( 125,255,255 ), blue_areas );
        Core.multiply( blue_areas.clone(), colorDiff, blue_areas );
        Imgproc.dilate( blue_areas.clone(), blue_areas, new Mat() );

        // There can be several blobs.  Find the largest that fills a certain amount
        // of the image.  These are crude heuristics but should be fine if we control
        // the conditions of when we start searching (ie, appx size of beacon in image
        // frame, etc).
        findWeightedPos(blue_areas, blue_blobs, blue_matches);
    }

    private void findRed()
    {
        // Same game, just a different hue
        Mat red1 = new Mat();
        Mat red2 = new Mat();
        Mat red_areas = new Mat();

        Core.inRange( zonedImg, new Scalar( 0,100,150 ), new Scalar( 10,255,255 ), red1);
        Core.inRange( zonedImg, new Scalar( 140,100,150 ), new Scalar( 179,255,255 ), red2);
        Core.bitwise_or(red1, red2, red_areas);
        Core.multiply( red_areas.clone(), colorDiff, red_areas );
        Imgproc.dilate( red_areas.clone(), red_areas, new Mat() );

        findWeightedPos( red_areas, red_blobs, red_matches );
    }

    public void findWeightedPos( Mat img, List<MatOfPoint> calcCtr, ArrayList<Rect> boxMatches ) {

        Mat hchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

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

        double contour_area, box_area;
        Rect bounded_box;

        calcCtr.clear();
        boxMatches.clear();

        each = contours.iterator();
        while (each.hasNext()) {

            MatOfPoint contour = each.next();

            contour_area = Imgproc.contourArea( contour );
            bounded_box = Imgproc.boundingRect( contour );

            if ( contour_area > MIN_COLOR_ZONE_AREA * maxArea ) {

                calcCtr.add(contour);
                boxMatches.add( bounded_box );

            }
        }
    }

    public Mat drawBeacon() {

        Mat out = image.clone();
        Imgproc.cvtColor( showImg, out, Imgproc.COLOR_HSV2RGB, 4 );
        if ( !sensingActive || !firstCalcsDone ) return out;

        for ( Rect bb : blue_matches )
        {
            Imgproc.rectangle( out, bb.tl(), bb.br(), new Scalar(150,150,255), -1 );
        }

        for ( Rect rb : red_matches )
        {
            Imgproc.rectangle( out, rb.tl(), rb.br(), new Scalar(255,150,150), -1 );
        }

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


    public LightOrder getLightOrder() {
        return LightOrder.UNKNOWN;
    }

}
