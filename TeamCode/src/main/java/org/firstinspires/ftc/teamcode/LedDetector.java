package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class LedDetector implements ImageProcessor
{
    private Mat hsvImage;
    private Mat zonedImg;
    private Mat showImg;

    private static final int LIGHT_CHANNEL = 1;
    private static final int A2RGB = Imgproc.COLOR_HLS2RGB;
    private static final int RGB2A = Imgproc.COLOR_RGB2HLS;
    private static final int MIN_AREA = 4;
    private static final int MIN_BRIGHTNESS = 220;
//    private static final int LIGHT_CHANNEL = 2;
//    private static final int A2RGB = Imgproc.COLOR_HSV2RGB;
//    private static final int RGB2A = Imgproc.COLOR_RGB2HSV;
//    private static final int MIN_BRIGHTNESS = 251;

    private List<Rect> leds = new ArrayList<>();
    //private RingBuffer beaconConfBuf = new RingBuffer(20);

    private boolean sensingActive = false;
    private boolean firstCalcsDone = false;

    private int numLeds = 0;

    private Telemetry telemetry = null;

    private int skipCnt = 0;
    private static final int SKIP = 50;

    static
    {
        if (!OpenCVLoader.initDebug())
        {
            DbgLog.error("SJH: OpenCVLoader error");
        }
    }

    @SuppressWarnings("WeakerAccess")
    public LedDetector() {}

    @Override
    public void startSensing()
    {
        firstCalcsDone = false;
        sensingActive = true;
    }

    public void stopSensing()
    {
        firstCalcsDone = false;
        sensingActive = false;
    }

    public void setImage( Mat img )
    {
        // Convert to HSV colorspace to make it easier to
        // threshold certain colors (ig red/blue)
        if(hsvImage == null)
        {
            hsvImage = new Mat();
        }
        Imgproc.cvtColor( img, hsvImage, RGB2A, 4 );
        showImg = hsvImage.clone();

        if ( !sensingActive ) return;
        findColors();
        firstCalcsDone = true;
    }

    public Mat draw()
    {
        Mat out = hsvImage.clone();
        Imgproc.cvtColor( showImg, out, A2RGB, 4 );
        if ( !sensingActive || !firstCalcsDone ) return out;

        for ( Rect ledRect : leds )
        {
            Imgproc.rectangle( out, ledRect.tl(), ledRect.br(), new Scalar(0,255,0), 4 );
        }

        return out;
    }

    public void logDebug()
    {
        DbgLog.msg("SJH: Num LEDs: %4d", numLeds);
    }

    public void setTelemetry(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    public void logTelemetry()
    {
        if(telemetry == null) return;

        telemetry.addData("#LEDs", "%4d", getNumLEDs());
    }

    public synchronized int getNumLEDs() { return numLeds; }

    private void findColors()
    {
        numLeds = 0;
        leds.clear();
        findLum();
        findLeds();
    }

    private void findLum()
    {
        List<Mat> channels = new ArrayList<>();
        Core.split( hsvImage, channels );

        Mat lum = channels.get( LIGHT_CHANNEL );

        Mat white = lum.clone();

        Imgproc.threshold( lum, white, MIN_BRIGHTNESS, 255, Imgproc.THRESH_BINARY );
        Imgproc.erode( white.clone(), white, Imgproc.getGaussianKernel( 5, 2 ) );

        channels.set( LIGHT_CHANNEL, white );

        if(zonedImg == null)
        {
            zonedImg = hsvImage.clone();
        }

        Core.merge( channels, zonedImg );

        //showImg = zonedImg.clone();
    }

    private void findLeds()
    {
        List<Mat> channels = new ArrayList<>();
        Core.split( zonedImg, channels );

        Mat d_value = channels.get( LIGHT_CHANNEL );

        Mat hchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(d_value, contours,
                             hchy, Imgproc.RETR_EXTERNAL,
                             Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        Rect bounded_box;

        skipCnt++;
        for (MatOfPoint wrapper : contours)
        {
            bounded_box = Imgproc.boundingRect(wrapper);

            if(bounded_box.area() < MIN_AREA) continue;

            leds.add(bounded_box);
            numLeds++;

            int ledX = (int)(bounded_box.x + bounded_box.width/2);
            int ledY = (int)(bounded_box.y + bounded_box.height/2);

            if(skipCnt % SKIP == 0)
            {
                DbgLog.msg("SJH : LED at %5d, %5d", ledX, ledY);
            }
        }
    }
}