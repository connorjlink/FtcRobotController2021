package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import java.lang.Math;

/*
    Basic driver control program. Inherits from opmode instead of linearopmode since we don't need the extra functionality, and we want to avoid having the linearopmode "stuck in stop()" issue.
*/
@TeleOp(name="MecanumDrive2021", group="TeleOP")
public class MecanumDrive extends OpMode
{
    public RobotHardware robot = null;

    @Override
    public void init()
    {
        robot = new RobotHardware(hardwareMap, false);
        telemetry.addData("Robot: ", "Initialization Complete");
    }

    //exponentially biases the input value on [0,1]. Can be used for the controller input if a pseudo "acceleration" is required. This makes the center of the joystick very precise, and the out edges very sensitive.
    public double bias(double x)
    {
        double biasFactor = 3.0;
        double val = ((Math.exp(Math.abs(x) * biasFactor) - 1) / (Math.exp(biasFactor) - 1));
        return (x > 0) ? val : -val;
    }

    @Override
    public void loop()
    {
        final double ROOT2 = 1.41421356237;

        //obtain the geometric configuration of the driver's gamepad joysticks
        final double magnitude = Math.hypot(bias(gamepad1.left_stick_x), bias(gamepad1.left_stick_y));
        final double robotAngle = Math.atan2(bias(-gamepad1.left_stick_y), bias(gamepad1.left_stick_x)) - Math.PI / 4;
        final double rightX = bias(gamepad1.right_stick_x) * 0.75;

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

        robot.setPowerAll(fld, frd, brd, bld);

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
        telemetry.update();


        //secondary driver runs the duck wheel so that the robot controller can align better
        robot.duckWheel.setPower(gamepad2.right_stick_x);

        //robot.tapeAngle.setPower(gamepad2.);
             if (gamepad2.dpad_left)  { robot.tapeExtend.setPower(-1.0); }
        else if (gamepad2.dpad_right) { robot.tapeExtend.setPower(1.0);  }
        else                          { robot.tapeExtend.setPower(0.0);  }


        robot.tapeAngle.setPower(bias(gamepad2.left_stick_x));


       // robot.slamra.getLastReceivedCameraUpdate();

        //selects purple box position (open/close) based on controller input
        if (gamepad2.dpad_down) { robot.intakeServo.setPosition(robot.INTAKE_DUMP);  }
        else                    { robot.intakeServo.setPosition(robot.INTAKE_STORE); }
    }
}