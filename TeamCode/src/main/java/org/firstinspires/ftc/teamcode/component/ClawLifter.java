package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawLifter
{
    public DcMotor clawLifter;
    public CRServo clawServo;

    public void onUpdate(Gamepad gamepad1, Gamepad gamepad2)
    {
        clawLifter.setPower(gamepad2.right_stick_y);
        clawServo.setPower(gamepad2.left_stick_y);
    }

    public ClawLifter(HardwareMap hardwareMap)
    {
        clawLifter = hardwareMap.dcMotor.get("clawLifter");
        clawServo = hardwareMap.crservo.get("clawServo");

        clawLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawLifter.setPower(0.0);

        clawServo.setPower(0.0);
    }
}
