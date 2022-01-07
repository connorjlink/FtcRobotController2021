package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="Test")
public class AutoTest extends AutonomousAbstract
{
    @Override
    public void runOpMode()
    {
        onInit("Shmizmin");



        //camera.detector.data = "R";

        //ElapsedTime time = new ElapsedTime();
        //time.reset();
//
        //while (opModeIsActive() && (time.seconds() < 10))
        //{
        //    telemetry.addData("Frame Count", camera.webcam.getFrameCount());
        //    telemetry.addData("FPS", String.format("%.2f", camera.webcam.getFps()));
        //    telemetry.addData("Total frame time ms", camera.webcam.getTotalFrameTimeMs());
        //    telemetry.addData("Pipeline time ms", camera.webcam.getPipelineTimeMs());
        //    telemetry.addData("Overhead time ms", camera.webcam.getOverheadTimeMs());
        //    telemetry.addData("Theoretical max FPS", camera.webcam.getCurrentPipelineMaxFps());
        //    telemetry.addData("Position", camera.detector.data);
        //    telemetry.addData("Right Total", camera.detector.right);
        //    telemetry.addData("Left Total", camera.detector.left);
        //    telemetry.update();
//
        //    sleep(100);
        //}
        //camera.webcam.stopStreaming();

        //rotate(90);
        //rotate(90);
        //rotate(-90);

        //for strafing, 40 inches code = 32.5 inches real

        //encoderDrive(-40.0, 40.0, 40.0, -40.0);
        //encoderDrive3(-15.0);
        // encoderDrive3(-15.0, -15.0, -15.0, -15.0);

        rotate(90);
        //rotate(-90);

        //camera.detector.data = "r";
        //switch (camera.detector.data)
        //{
        //    case "l":
        //    case "c":
        //    case "r":
        //
        //        break;
        //    //case "c":
//
////
        //    //case "r":
        //    //    //rotate(90, robot.HALF_POWER);
        //    //    encoderDrive(40.0, 40.0, 40.0, 40.0);
        //    //    break;
        //}
    }

}