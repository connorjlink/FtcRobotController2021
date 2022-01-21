package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BlockLifter
{
    public DcMotor blockLifter;
    public Servo blockDropper;

    public void onUpdate(Gamepad gamepad1, Gamepad gamepad2)
    {
        blockLifter.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        if (gamepad2.dpad_down) { blockDropper.setPosition(1.0);  }
        else                    { blockDropper.setPosition(0.0); }
    }

    public BlockLifter(HardwareMap hardwareMap)
    {
        blockLifter = hardwareMap.dcMotor.get("intakeLifter");
        blockDropper = hardwareMap.servo.get("intakeServo");

        blockLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blockLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blockLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blockLifter.setPower(0.0);

        blockDropper.setPosition(0.0);
    }
}
