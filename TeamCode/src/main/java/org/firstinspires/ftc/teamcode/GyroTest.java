package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Martin on 11/9/2016.
 */
@Autonomous(name="Gyro Test", group="Auton")
public class GyroTest extends LinearOpMode
{
    public void runOpMode() throws InterruptedException
    {
        robot.init(this);

        waitForStart();

     //   move(.3, -.3);

        while(opModeIsActive())
        {
            telemetry.addData("Gyro Heading : ", robot.gyro.getHeading());
            telemetry.update();

            Thread.sleep(10);
            idle();
        }

    }
    private ShelbyBot robot = new ShelbyBot();

    void move(double ldp, double rdp)
    {
        robot.leftMotor.setPower(ldp);
        robot.rightMotor.setPower(rdp);
    }

}
