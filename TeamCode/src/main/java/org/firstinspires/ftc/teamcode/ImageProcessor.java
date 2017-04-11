package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

public interface ImageProcessor
{
    public void startSensing();
    public void stopSensing();

    public void logDebug();
    public void logTelemetry();
    public void setTelemetry(Telemetry telemetry);

    public void setImage( Mat img );
    public void snapImage(int imgNum);
    public void saveImage(int imgNum);
    public Mat draw();
}
