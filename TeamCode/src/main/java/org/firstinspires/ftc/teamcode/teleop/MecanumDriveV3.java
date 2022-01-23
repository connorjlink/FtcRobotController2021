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
@TeleOp(name="Mecanum Super Qualifier", group="TeleOp")
public class MecanumDriveV3 extends OpMode
{
    private RobotV3 robot = new RobotV3(hardwareMap);
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

        telemetry.update();
    }
}