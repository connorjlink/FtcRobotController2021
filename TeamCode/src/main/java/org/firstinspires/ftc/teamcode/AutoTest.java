package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test")
public class AutoTest extends AutonomousAbstract
{
    @Override
    public void runOpMode()
    {
        onInit("Shmizmin");
        AutonomousTimeout.reset();

        webcam.stopStreaming();

        detector.data = "c";

        switch (detector.data)
        {
            case "l":
            case "c":
            case "r":
                //rotate(90, robot.HALF_POWER);
                robot.DRIVE_SPEED = 0.25;
                encoderDrive(40.0, 40.0, 40.0, 40.0);
        }
    }

}