package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class VuforiaTest
{
    public VuforiaTest()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);

        //SJH Teams license
        parameters.vuforiaLicenseKey = "AQgIvJ7/////AAAAGQSociXWO0kDvfP15zd4zOsS+fHJygDMLA" +
                "1HhOJQ3FkeiPLGU6YW3ru+jzC6MGxM5tY1ajF4Y0plOpxhQGfS" +
                "R4g3zFiP0IQavezWhGbjBCRMmYu8INy8KvoZ03crZe9wxxQJu9" +
                "9KiNX3ZrbUevNXODKKzWyA9RqxxQHbJ3gpXoff4z1O9n211VOg" +
                "EsJjrNZq8xJnznilyXwc8colJnZD/Adr6UmOzxoUGgaMrdPrlj" +
                "McDJZU6uyoIrOjiv1G2r3iNjtd7LzKAANKrK/0IrO90MgRqQDr" +
                "CAAJVHqqyyubMy8EqE5onzw/WFEcEwfQ6nolsNwYTEZb/JppU8" +
                "9Q6DZmhz4FCT49shA+4PyNOzqsjhRC";
        //S

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    }

    public String getLic()
    {
        return parameters.vuforiaLicenseKey;
    }

    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters =
            new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);

    public static void main(String args[])
    {
        VuforiaTest v = new VuforiaTest();
        String lk = v.getLic();

    }
}
