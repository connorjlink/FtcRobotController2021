package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;


/*
    Basic driver control program.
    Inherits from opmode instead of linearopmode since we don't need the extra functionality.
    This also avoids the linearopmode "stuck in stop()" issue.
*/
@TeleOp(name="Mecanum League Tournament", group="TeleOp")
public class MecanumDriveV2 extends OpMode
{
    private RobotV2 robot = new RobotV2(hardwareMap);
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init()
    {
        telemetry.addLine("Robot Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        robot.onUpdate();

        telemetry.addData("clawLifter", robot.clawLifter.getCurrentPosition());
        telemetry.addData("intakeLifter", robot.intakeLifter.getCurrentPosition());
        telemetry.addData("fl", robot.frontLeftDrive.getCurrentPosition());
        telemetry.addData("fr", robot.frontRightDrive.getCurrentPosition());
        telemetry.addData("bl", robot.backLeftDrive.getCurrentPosition());
        telemetry.addData("br", robot.backRightDrive.getCurrentPosition());

        telemetry.update();
    }
}