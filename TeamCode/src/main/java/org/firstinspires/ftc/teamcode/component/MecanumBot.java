package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class MecanumBot
{
    public  DcMotorEx frontLeftDrive  = null,
                      frontRightDrive = null,
                      backLeftDrive   = null,
                      backRightDrive  = null;



    private static final double ROOT2 = Math.sqrt(2.0);

    public void setMode(DcMotor.RunMode runMode)
    {
        frontLeftDrive.setMode(runMode);
        frontRightDrive.setMode(runMode);
        backLeftDrive.setMode(runMode);
        backRightDrive.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        frontLeftDrive.setZeroPowerBehavior(zeroPowerBehavior);
        frontRightDrive.setZeroPowerBehavior(zeroPowerBehavior);
        backLeftDrive.setZeroPowerBehavior(zeroPowerBehavior);
        backRightDrive.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPowerLeft(double power)
    {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
    }

    public void setPowerRight(double power)
    {
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void setPower(double power)
    {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void setPower(double fld, double frd, double bld, double brd)
    {
        frontLeftDrive.setPower(fld);
        frontRightDrive.setPower(frd);
        backLeftDrive.setPower(bld);
        backRightDrive.setPower(brd);
    }

    //exponentially biases the input value on [0,1]. Can be used for the controller input if a pseudo "acceleration" is required. This makes the center of the joystick very precise, and the out edges very sensitive.
    //particularly useful for sensitive controls when lining up the duck wheel mechanism, loading freight on the top level, and capping the team element
    private double bias(double x)
    {
        double biasFactor = 3.0;
        double val = ((Math.exp(Math.abs(x) * biasFactor) - 1) / (Math.exp(biasFactor) - 1));
        return (x > 0) ? val : -val;
    }

    //controls the robot based on the gamepad input
    public void onUpdate(Gamepad gamepad1, Gamepad gamepad2)
    {
        boolean wrong = false;
        double magnitude, robotAngle, rightX;


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

        setPower(fld, frd, brd, bld);
    }

    public MecanumBot(HardwareMap hardwareMap)
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setPower(0.0);
    }
}
