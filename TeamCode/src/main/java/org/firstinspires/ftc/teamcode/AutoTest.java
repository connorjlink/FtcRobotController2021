package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="Test")
public class AutoTest extends AutonomousAbstract
{
    @Override
    public void runOpMode()
    {
        onInit("shmizmin");

        AutonomousTimeout.reset();

        while (opModeIsActive() && (AutonomousTimeout.milliseconds() < 5000))
        {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Position", detector.data);
            telemetry.addData("Right Total", detector.right);
            telemetry.addData("Left Total", detector.left);
            telemetry.update();

            sleep(100);
        }

        webcam.stopStreaming();

        if (detector.data == "c" || detector.data == "r")
        {
            //load block from green wheel into stacker
            robot.clawServo.setPower(-1.0);
            sleep(1000);
            robot.clawServo.setPower(0.0);
        }

        switch (detector.data)
        {
            //block is left, needs to go to lowest
            case "l":
                encoderDrive(robot.DRIVE_SPEED / 2.0, -6.0, -6.0, -6.0, -6.0, 1000);
                encoderDrive(robot.DRIVE_SPEED / 2.0, 4.0, -4.0, 4.0, -4.0, 1000);
                encoderDrive(robot.DRIVE_SPEED / 2.0, -13.0, -13.0, -13.0, -13.0, 1000);
                encoderDrive(robot.DRIVE_SPEED / 2.0, 20.0, -20.0, 20.0, -20.0, 1000);

                robot.clawLifter.setPower(-0.45);
                sleep(1000);
                robot.clawLifter.setPower(0.0);

                robot.clawServo.setPower(-1.0);
                sleep(1250);
                robot.clawServo.setPower(0.0);

                robot.clawLifter.setPower(1.0);
                sleep(800);
                robot.clawLifter.setPower(0.0);

                encoderDrive(robot.DRIVE_SPEED / 2.0, -4.0, -4.0, -4.0, -4.0, 1000);
                encoderDrive(robot.DRIVE_SPEED / 2.0, 6.0, -6.0, 6.0, -6.0, 1000);
                encoderDrive(robot.DRIVE_SPEED, 50.0, 50.0, 50.0, 50.0, 1000);

                break;
                //block is center, needs to go to middle
            case "c":
                encoderDrive(robot.DRIVE_SPEED / 2.0, -6.0, -6.0, -6.0, -6.0, 1000);
                encoderDrive(robot.DRIVE_SPEED / 2.0, 4.0, -4.0, 4.0, -4.0, 1000);
                encoderDrive(robot.DRIVE_SPEED / 2.0, -12.0, -12,-12.0, -12.0, 1000);
                //encoderDrive(robot.DRIVE_SPEED / 2.0, 20.0, -20.0, 20.0, -20.0, 1000);

                robot.intakeLifter.setPower(1.0);
                sleep(1000);
                robot.intakeLifter.setPower(0.0);
                sleep(250);
                robot.intakeServo.setPosition(robot.INTAKE_DUMP);
                sleep(2000);
                robot.intakeServo.setPosition(robot.INTAKE_STORE);
                sleep(1000);
                robot.intakeLifter.setPower(-1.0);
                sleep(400);
                robot.intakeLifter.setPower(0.0);

                encoderDrive(robot.DRIVE_SPEED / 2.0, 4.0, 4.0, 4.0, 4.0, 1000);
                encoderDrive(robot.DRIVE_SPEED / 2.0, 6.5, -6.5, 6.5, -6.5, 1000);
                encoderDrive(robot.DRIVE_SPEED, -50.0, -50.0, -50.0, -50.0, 1000);

                //encoderDrive(robot.DRIVE_SPEED / 2.0, -4.0, 4.0, -4.0, 4.0, 1000);
                break;
                //block is right, needs to go to top
            case "r":
                encoderDrive(robot.DRIVE_SPEED / 2.0, -6.0, -6.0, -6.0, -6.0, 1000);
                encoderDrive(robot.DRIVE_SPEED / 2.0, 4.0, -4.0, 4.0, -4.0, 1000);
                encoderDrive(robot.DRIVE_SPEED / 2.0, -13.0, -13.0, -13.0, -13.0, 1000);

                robot.intakeLifter.setPower(1.0);
                sleep(1000);
                robot.intakeLifter.setPower(0.0);
                sleep(250);
                robot.intakeServo.setPosition(robot.INTAKE_DUMP);
                sleep(2000);
                robot.intakeServo.setPosition(robot.INTAKE_STORE);
                sleep(1000);
                robot.intakeLifter.setPower(-1.0);
                sleep(400);
                robot.intakeLifter.setPower(0.0);

                encoderDrive(robot.DRIVE_SPEED / 2.0, 4.0, 4.0, 4.0, 4.0, 1000);
                encoderDrive(robot.DRIVE_SPEED / 2.0, 6.5, -6.5, 6.5, -6.5, 1000);
                encoderDrive(robot.DRIVE_SPEED, -50.0, -50.0, -50.0, -50.0, 1000);

                //encoderDrive(robot.DRIVE_SPEED / 2.0, 4.0, -4.0, 4.0, -4.0, 1000);
                break;
        }
    }

}