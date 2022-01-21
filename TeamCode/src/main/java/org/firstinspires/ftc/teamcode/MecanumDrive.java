package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.RobotHardware;
import java.lang.Math;

/*
    Basic driver control program. Inherits from opmode instead of linearopmode since we don't need the extra functionality, and we want to avoid having the linearopmode "stuck in stop()" issue.
*/
@TeleOp(name="MecanumDrive2021", group="TeleOP")
public class MecanumDrive extends OpMode
{
    public RobotHardware robot = null;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private MecanumBot mecanumBot;
    private ClawLifter clawLifter;
    private BlockLifter blockLifter;

    @Override
    public void init()
    {
        robot = new RobotHardware(hardwareMap, false);
        telemetry.addData("Robot: ", "Initialization Complete");

        mecanumBot = new MecanumBot(hardwareMap);
        clawLifter = new ClawLifter(hardwareMap);
        blockLifter = new BlockLifter(hardwareMap);
    }

    //exponentially biases the input value on [0,1]. Can be used for the controller input if a pseudo "acceleration" is required. This makes the center of the joystick very precise, and the out edges very sensitive.
    //particularly useful for sensitive controls when lining up the duck wheel mechanism, loading freight on the top level, and capping the team element
    public double bias(double x)
    {
        double biasFactor = 3.0;
        double val = ((Math.exp(Math.abs(x) * biasFactor) - 1) / (Math.exp(biasFactor) - 1));
        return (x > 0) ? val : -val;
    }

    @Override
    public void loop()
    {
        mecanumBot.onUpdate(gamepad1, gamepad2);
        clawLifter.onUpdate(gamepad1, gamepad2);
        blockLifter.onUpdate(gamepad1, gamepad2);


        //for some reason, the controllers hardware became switched around--might need to revert to this if teleop stops working correctly
        //final double magnitude = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        //final double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        //final double rightX = gamepad1.right_stick_x;

        //perform the geometric vector addition using trig functions, and correct the resultant value
        //using this method, the maximal magnitude that could be reached is sqrt(2)/2, so multiply by sqrt(2) to get a maximum of 1
        final double fld = (magnitude * Math.cos(robotAngle) + rightX) * ROOT2;
        final double frd = (magnitude * Math.sin(robotAngle) - rightX) * ROOT2;
        final double brd = (magnitude * Math.sin(robotAngle) + rightX) * ROOT2;
        final double bld = (magnitude * Math.cos(robotAngle) - rightX) * ROOT2;

        robot.setPowerAll(fld, frd, bld, brd);

        //obtain the difference between the gamepad trigger to lift the claw
        //right trigger moves it up, left trigger moves it down, both together doesn't move it
        robot.intakeLifter.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        //switch direction to make operating the arm more intuitive, and change the power depending on which direction is requested so the arm doesn't slam down and break
        //robot.clawLifter.setPower(gamepad2.right_stick_y > 0 ? -gamepad2.right_stick_y * 0.5 : -gamepad2.right_stick_y);


        //robot.ARM_TARGET += (gamepad2.right_stick_y / 10);
        //robot.clawLifter.setTargetPosition((int)robot.ARM_TARGET);
        //robot.clawLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.clawLifter.setPower(0.8);

        robot.clawLifter.setPower(gamepad2.right_stick_y);
        robot.clawServo.setPower(gamepad2.left_stick_y);

        //robot.clawLifter.setTargetPosition((int)robot.ARM_TARGET);
        //robot.ARM_TARGET += (gamepad2.right_stick_y * 10.0);
        telemetry.addData("clawLifter", robot.clawLifter.getCurrentPosition());
        telemetry.addData("intakeLifter", robot.intakeLifter.getCurrentPosition());
        telemetry.addData("fl", robot.frontLeftDrive.getCurrentPosition());
        telemetry.addData("fr", robot.frontRightDrive.getCurrentPosition());
        telemetry.addData("bl", robot.backLeftDrive.getCurrentPosition());
        telemetry.addData("br", robot.backRightDrive.getCurrentPosition());


        //secondary driver runs the duck wheel so that the robot controller can align better
        robot.duckWheel.setPower(gamepad2.right_stick_x);

        //robot.tapeAngle.setPower(gamepad2.);
             if (gamepad2.dpad_left)  { robot.tapeExtend.setPower(-1.0); }
        else if (gamepad2.dpad_right) { robot.tapeExtend.setPower(1.0);  }
        else                          { robot.tapeExtend.setPower(0.0);  }


        robot.tapeAngle.setPower(bias(gamepad2.left_stick_x));

        //selects purple box position (open/close) based on controller input
        if (gamepad2.dpad_down) { robot.intakeServo.setPosition(robot.INTAKE_DUMP);  }
        else                    { robot.intakeServo.setPosition(robot.INTAKE_STORE); }

        //{
        //    final int robotRadius = 11; // inches

        //    TelemetryPacket packet = new TelemetryPacket();
        //    Canvas field = packet.fieldOverlay();

        //    T265Camera.CameraUpdate up = robot.slamra.slamra.getLastReceivedCameraUpdate();
        //    if (up == null)
        //    {
        //        telemetry.addLine("NULL");
        //        telemetry.update();
        //        return;
        //    }

        //    // We divide by 0.0254 to convert meters to inches
        //    Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        //    Rotation2d rotation = up.pose.getRotation();

        //    field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        //    double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        //    double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        //    double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        //    field.strokeLine(x1, y1, x2, y2);

        //    telemetry.addData("Robot X", up.pose.getTranslation().getX() / 0.0254);
        //    telemetry.addData("Robot Y", up.pose.getTranslation().getY() / 0.0254);
        //    telemetry.addData("Robot Z", up.pose.getRotation().getDegrees());

        //    dashboard.sendTelemetryPacket(packet);
        //}

        telemetry.update();
    }
}