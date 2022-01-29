package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.RobotV2;

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
        robot.onUpdate(gamepad1, gamepad2);
    }
}