package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.lang.Math;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode
{
    //exponentially biases the input value on [0,1]. Can be used for the controller input if a pseudo "acceleration" is required. This makes the center of the joystick very precise, and the out edges very sensitive.
    //particularly useful for sensitive controls when lining up the duck wheel mechanism, loading freight on the top level, and capping the team element
    private double bias(double x)
    {
        double biasFactor = 3.0;
        double val = ((Math.exp(Math.abs(x) * biasFactor) - 1) / (Math.exp(biasFactor) - 1));
        return (x > 0) ? val : -val;
    }

    private static final double ROOT2 = Math.sqrt(2.0);

    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap);

        //FtcDashboard dashboard = FtcDashboard.getInstance();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            //-gamepad1.right_stick_x,
                            //-gamepad1.left_stick_x,
                            //-gamepad1.left_stick_y
                            0.0,0.0,0.0
                    )
            );

            drive.update();

            drive.setLocalizer(localizer);

            //{
            boolean wrong = true;
            double magnitude, robotAngle, rightX;

            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //even though the wires are in the correct spot on the rewired robot, there a glitch somewhere in the system that flips the right and left analog stick X sensors
            //this code just reverses that so it can drive normally in either circumstance, correct or incorrect
            if (wrong)
            {
                //obtain the geometric configuration of the driver's gamepad joysticks
                magnitude = Math.hypot(bias(gamepad1.right_stick_x), bias(gamepad1.left_stick_y));
                robotAngle = Math.atan2(bias(-gamepad1.left_stick_y), bias(gamepad1.right_stick_x)) - Math.PI / 4;
                rightX = gamepad1.left_stick_x * 0.5;
            }

            else
            {
                //obtain the geometric configuration of the driver's gamepad joysticks
                magnitude = Math.hypot(bias(gamepad1.left_stick_x), bias(gamepad1.left_stick_y));
                robotAngle = Math.atan2(bias(-gamepad1.left_stick_y), bias(gamepad1.left_stick_x)) - Math.PI / 4;
                rightX = gamepad1.right_stick_x * 0.5;
            }

            final double fld = (magnitude * Math.cos(robotAngle) + rightX) * ROOT2;
            final double frd = (magnitude * Math.sin(robotAngle) - rightX) * ROOT2;
            final double brd = (magnitude * Math.sin(robotAngle) + rightX) * ROOT2;
            final double bld = (magnitude * Math.cos(robotAngle) - rightX) * ROOT2;

            drive.setMotorPowers(fld, frd, bld, brd);

           //{
           //    final int robotRadius = 11; // inches
////
           //    TelemetryPacket packet = new TelemetryPacket();
           //    Canvas field = packet.fieldOverlay();
//
           //    localizer.update();
           //    Pose2d poseEstimate = localizer.getPoseEstimate();
           //    double x = DriveConstants.encoderTicksToInches(poseEstimate.getX());
           //    double y = DriveConstants.encoderTicksToInches(poseEstimate.getY());
           //    double ux = poseEstimate.getX();
           //    double uy = poseEstimate.getY();
           //    double z = poseEstimate.getHeading() * (180.0 / Math.PI);
//
           //    // We divide by 0.0254 to convert meters to inches
           //    Translation2d translation = new Translation2d(ux, uy);
           //    Rotation2d rotation = new Rotation2d(z);
//
           //    field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
           //    double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
           //    double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
           //    double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
           //    field.strokeLine(x1, y1, x2, y2);
//
           //    telemetry.addData("Robot Z", z);
           //    telemetry.addData("Robot UX", ux);
           //    telemetry.addData("Robot UY", uy);
           //    telemetry.addData("UV", localizer.frontEncoder.getCurrentPosition());
           //    telemetry.update();
//
           //    dashboard.sendTelemetryPacket(packet);
           //}

        }
    }
}
