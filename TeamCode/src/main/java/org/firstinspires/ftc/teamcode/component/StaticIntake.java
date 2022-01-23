package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StaticIntake
{
    private DcMotor staticIntake;

    public void onUpdate(Gamepad gamepad1, Gamepad gamepad2)
    {
        blockLifter.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }

    public StaticIntake(HardwareMap hardwareMap)
    {
        staticIntake = hardwareMap.dcmotor.get("staticIntake");

        staticIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        staticIntake.setPower(0.0);
    }
}
