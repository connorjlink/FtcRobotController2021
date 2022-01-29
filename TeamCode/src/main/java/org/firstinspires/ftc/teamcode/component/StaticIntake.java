package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StaticIntake
{
    private DcMotor staticIntake;

    public void onUpdate(Gamepad gamepad1, Gamepad gamepad2)
    {
        staticIntake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }

    public StaticIntake(HardwareMap hardwareMap)
    {
        staticIntake = hardwareMap.get(DcMotor.class, "staticIntake");

        staticIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        staticIntake.setPower(0.0);
    }
}
