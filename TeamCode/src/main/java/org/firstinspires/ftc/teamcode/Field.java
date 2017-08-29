package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Field
{
    enum Alliance {BLUE, RED}

    enum AutoStrategy
    {
        SHOOT_PUSH_PARK,
        SHOOT_PARK
    }

    enum StartPos
    {
        START_A_SWEEPER,
        START_B_SWEEPER,
        START_R_PUSHER,
        START_TEST
    }

    enum BeaconChoice
    {
        BOTH,
        NEAR,
        FAR,
        NONE
    }

    enum ParkChoice
    {
        CENTER_PARK,
        CORNER_PARK,
        DEFEND_PARK
    }

    //  X axis parallel to red  alliance wall point toward    blue alliance
    //  Y axis parallel to blue alliance wall point away from red  alliance

    //The descriptions say the field is 12'x12', but our
    //practice field is actually slightly smaller at 141"
    private static final float X_WIDTH = 141.0f;
    private static final float Y_WIDTH = 141.0f;
    static final float N_WALL_Y = Y_WIDTH/2.0f;
    static final float E_WALL_X = X_WIDTH/2.0f;
    static final float S_WALL_Y = -N_WALL_Y;
    static final float W_WALL_X = -E_WALL_X;

    private static final float MM_PER_INCH = 25.4f;
    private static final float IMAGE_Z = 6.50f;

    static final float[] toolsPos  = {W_WALL_X,  3.0f*12, IMAGE_Z};
    static final float[] gearsPos  = {W_WALL_X, -1.0f*12, IMAGE_Z};
    static final float[] wheelsPos = { 1.0f*12, N_WALL_Y, IMAGE_Z};
    static final float[] legosPos  = {-3.0f*12, N_WALL_Y, IMAGE_Z};
    //static final float[] legosPos  = {0.0f*12, 0.0f, 0.0f};

    private static final float[] toolsRot  = {90.0f, 0.0f, 90.0f};
    private static final float[] gearsRot  = {90.0f, 0.0f, 90.0f};
    private static final float[] wheelsRot = {90.0f, 0.0f,  0.0f};
    private static final float[] legosRot  = {90.0f, 0.0f,  0.0f};
    //private static final float[] legosRot  = {0.0f, 0.0f,  0.0f};

    private static final float[] toolsPosMm  = scaleArr(toolsPos,  MM_PER_INCH);
    private static final float[] gearsPosMm  = scaleArr(gearsPos,  MM_PER_INCH);
    private static final float[] wheelsPosMm = scaleArr(wheelsPos, MM_PER_INCH);
    private static final float[] legosPosMm  = scaleArr(legosPos,  MM_PER_INCH);

    static final OpenGLMatrix redToolsLocationOnField   = genMatrix(toolsPosMm,  toolsRot);
    static final OpenGLMatrix redGearsLocationOnField   = genMatrix(gearsPosMm,  gearsRot);
    static final OpenGLMatrix blueWheelsLocationOnField = genMatrix(wheelsPosMm, wheelsRot);
    static final OpenGLMatrix blueLegosLocationOnField  = genMatrix(legosPosMm,  legosRot);

    private static float[] scaleArr(float[] inArr, float scale)
    {
        float[] outArr = {0.0f, 0.0f, 0.0f};
        for (int i =0; i<inArr.length; ++i)
        {
            outArr[i] = inArr[i] * scale;
        }
        return outArr;
    }

    private static OpenGLMatrix genMatrix(float[] pos, float[] rot)
    {
        return OpenGLMatrix
                .translation(pos[0], pos[1], pos[2])
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                        rot[0], rot[1], rot[2]));
    }
}
