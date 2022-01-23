package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DuckWheel
{
    private DcMotor duckWheel;

    public void onUpdate(Gamepad gamepad1, Gamepad gamepad2)
    {
        duckWheel.setPower(gamepad2.right_stick_x);
    }

    public DuckWheel(HardwareMap hardwareMap)
    {
        duckWheel = hardwareMap.dcmotor.get("duckWheel");

        duckWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckWheel.setPower(0.0);
    }
}