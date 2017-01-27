package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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


public class BeaconDetector implements BeaconFinder, ImageProcessor
{
    private final static double MIN_COLOR_ZONE_AREA = 0.2; 	    // fraction of total image area

    private Mat hsvImg;
    private Mat zonedImg;
    private Mat tmpHsvImg;
    private Mat tmp1Img;
    private Mat maskImg;
    private Mat showImg;
    private Mat cvImage;
    private Mat colorDiff;
    private Mat onesImg;
    private Mat zeroImg;
    private Mat white;
    private Mat out;

    private final static boolean DEBUG = false;
    private final static boolean POS_IS_Y = false;

    private Telemetry telemetry = null;

    private boolean channels_initialized = false;

    private List<Mat>        hsv_channels = new ArrayList<>();
    private List<Mat>        rgb_channels = new ArrayList<>();
    private List<MatOfPoint> red_blobs    = new ArrayList<>();
    private List<MatOfPoint> blue_blobs   = new ArrayList<>();
    private List<MatOfPoint> white_blobs  = new ArrayList<>();
    private List<MatOfPoint> black_blobs  = new ArrayList<>();

    private ArrayList<Rect> red_matches   = new ArrayList<>();
    private ArrayList<Rect> blue_matches  = new ArrayList<>();
    private ArrayList<Rect> white_matches = new ArrayList<>();
    private List<Rect> buttons            = new ArrayList<>();

    private Rect red_box;
    private Rect blue_box;
    private Rect white_box;
    private Rect beacon_box;

    private RingBuffer beaconConfBuf = new RingBuffer(20);
    private RingBuffer beaconPosXBuf = new RingBuffer(3);
    private RingBuffer beaconPosZBuf = new RingBuffer(2);

    private double beaconConf = 0;
    private double beaconPosX = 0;
    private double beaconPosZ = 0;

    private BeaconSide redPosSide  = BeaconSide.UNKNOWN;
    private BeaconSide bluePosSide = BeaconSide.UNKNOWN;

    private boolean sensingActive = false;
    private boolean firstCalcsDone = false;

    static
    {
        if (!OpenCVLoader.initDebug()) {
            DbgLog.error("SJH: OpenCVLoader error"); //Handle opencv loader issue
        }
    }

    @SuppressWarnings("WeakerAccess")
    public BeaconDetector(Mat img ) {
        this();
        setImage( img );
    }

    public BeaconDetector()
    {
    }

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
        if(hsvImg == null) hsvImg = new Mat();

        Imgproc.cvtColor( img, hsvImg, Imgproc.COLOR_RGB2HSV, 4 );

        if(showImg == null) showImg = hsvImg.clone();
        else hsvImg.copyTo(showImg);

        if(!channels_initialized)
        {
           channels_initialized = true;
           for(int c = 0; c < img.channels(); c++)
           {
              rgb_channels.add(new Mat());
           }

           for(int c = 0; c < hsvImg.channels(); c++)
           {
              hsv_channels.add(new Mat());
           }
        }

        Core.split( img, rgb_channels );
        Mat red = rgb_channels.get( 0 );
        Mat blue = rgb_channels.get( 2 );

        if(colorDiff == null) colorDiff = new Mat(red.rows(), red.cols(), red.type());

        Core.absdiff( red, blue, colorDiff );

        if(tmp1Img == null) tmp1Img = colorDiff.clone();
        else colorDiff.copyTo(tmp1Img);

        Imgproc.threshold( tmp1Img,  colorDiff, 20, 255, Imgproc.THRESH_BINARY );

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

    public void logTelemetry()
    {
        if(telemetry == null) return;

        telemetry.addData( "CONF", "%5.2f", getBeaconConf() );
        telemetry.addData( "X", "%5.2f",  getBeaconPosX() );
        telemetry.addData( "Z", "%5.2f", getBeaconPosZ() );
        telemetry.addData( "RED", "%s",  getRedPosSide() );
        telemetry.addData( "BLUE", "%s", getBluePosSide() );
    }

    public void setTelemetry(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    public synchronized BeaconSide getRedPosSide() { return redPosSide; }
    public synchronized BeaconSide getBluePosSide() { return bluePosSide; }
    public synchronized double getBeaconConf() { return beaconConf; }
    public synchronized double getBeaconPosX() { return beaconPosX; }
    public synchronized double getBeaconPosZ() { return beaconPosZ; }

    private double scoreFit( Rect wb, Rect rb, Rect bb )
    {
        double beac_w = 8.5;
        double beac_h = 6.5;

        double actl_beac_rt = beac_w / beac_h;
        double sens_bcn_rt = wb.width / wb.height;

        double beac_rt = wb.area() / ( hsvImg.cols() * hsvImg.rows() );
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
                redPosSide  = BeaconSide.UNKNOWN;
                bluePosSide = BeaconSide.UNKNOWN;
            }

            return;
        }

        double scrn_ctr = hsvImg.cols() / 2;
        double beac_ctr = beacon_box.x + beacon_box.width / 2;

        beaconConf = beaconConfBuf.smooth( scoreFit( beacon_box, red_box, blue_box ) );
        beaconPosX = beaconPosXBuf.smooth( beac_ctr - scrn_ctr );
        beaconPosZ = beaconPosZBuf.smooth( (double) beacon_box.width / (double) hsvImg.cols() );

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
            redPosSide  = BeaconSide.UNKNOWN;
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

    private void findLum()
    {
        Core.split( hsvImg, hsv_channels );

        Mat sat = hsv_channels.get( 1 );
        Mat lum = hsv_channels.get( 2 );

        double lumAvg = Core.mean( lum ).val[0];

        if(tmp1Img == null) tmp1Img = sat.clone();
        else sat.copyTo(tmp1Img);

        Core.normalize( tmp1Img, sat, 120, 255, Core.NORM_MINMAX );
        lum.copyTo(tmp1Img);
        Core.normalize( tmp1Img, lum,   0, 180, Core.NORM_MINMAX );

        if(white == null) white = lum.clone();

        Imgproc.GaussianBlur( lum, tmp1Img, new Size(25,25), 25);
        Imgproc.threshold( tmp1Img, white, 255 - lumAvg, 255, Imgproc.THRESH_BINARY ); 
        //+ or Imgproc.THRESH_OTSU
        white.copyTo(tmp1Img);
        Imgproc.erode( tmp1Img, white, Imgproc.getGaussianKernel( 5, 2 ) );

        findWeightedPos( white, white_blobs, white_matches );

        hsv_channels.set( 1, sat );
        hsv_channels.set( 2, lum );

        if(zonedImg == null)  zonedImg = new Mat(hsvImg.rows(), hsvImg.cols(), hsvImg.type());

        Core.merge( hsv_channels, zonedImg );

        if(tmpHsvImg == null)  tmpHsvImg = zonedImg.clone();
        else                   zonedImg.copyTo(tmpHsvImg);

        List<Mat> tmp = new ArrayList<>();
        Core.split( hsvImg, tmp );

        if(maskImg == null)  maskImg = new Mat(hsvImg.rows(), hsvImg.cols(), hsvImg.type());

        if(onesImg == null)  
           onesImg = Mat.ones( hsvImg.rows(), hsvImg.cols(), white.type() );
        if(zeroImg == null)  
           zeroImg = Mat.zeros( hsvImg.rows(), hsvImg.cols(), white.type() );

        white.copyTo(tmp1Img);
        Imgproc.dilate( tmp1Img, white, Imgproc.getGaussianKernel( 5, 2 ) );
        tmp.set( 0, onesImg );
        tmp.set( 1, onesImg );
        tmp.set( 2, white );
        Core.merge( tmp, maskImg );

        Core.multiply( tmpHsvImg, maskImg, zonedImg );
        zonedImg.copyTo(showImg);
    }

    private void findButtons()
    {
        buttons.clear();
        black_blobs.clear();

        Core.split( zonedImg, hsv_channels );

        Mat d_value = hsv_channels.get( 2 );
        d_value.copyTo(tmp1Img);
        //Mat inv = new Mat( d_value.rows(), d_value.cols(), d_value.type(), new Scalar( 255 ) );
        //Imgproc.adaptiveThreshold( tmp1Img, d_value, 255.0, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, 15, 0 );
        //d_value.copyTo(tmp1Img);
        //Core.subtract( inv, tmp1Img, d_value );
        //d_value.copyTo(tmp1Img);
        //Imgproc.GaussianBlur( tmp1Img, d_value, new Size(15,15), 15);
        //d_value.copyTo(tmp1Img);
        Imgproc.threshold( tmp1Img, d_value, 40, 255, Imgproc.THRESH_BINARY_INV ); // + Imgproc.THRESH_OTSU );

//        hsv_channels.set( 0, zeroImg);
//        hsv_channels.set( 1, zeroImg);
//        hsv_channels.set( 2, d_value );
//
//        Core.merge( hsv_channels, showImg );

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
        blue_areas.copyTo(tmpHsvImg);
        Core.multiply( tmpHsvImg, colorDiff, blue_areas );
        blue_areas.copyTo(tmpHsvImg);
        Imgproc.dilate( tmpHsvImg, blue_areas, new Mat() );

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
        red_areas.copyTo(tmpHsvImg);
        Core.multiply( tmpHsvImg, colorDiff, red_areas );
        red_areas.copyTo(tmpHsvImg);
        Imgproc.dilate( tmpHsvImg, red_areas, new Mat() );

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

    public Mat draw() {

        if(out == null) out = hsvImg.clone();
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

        if (cvImage == null)  cvImage  = new Mat(inHeight, inWidth, cvt);
        if (hsvImg == null) hsvImg = new Mat(inHeight, inWidth, cvt);

        Utils.bitmapToMat(rgbImage, cvImage);
        setImage(cvImage);
    }


    public LightOrder getLightOrder() {
        return LightOrder.UNKNOWN;
    }

}


