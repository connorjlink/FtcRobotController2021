package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MeasureTape
{
    private CRServo tapeAngle;
    private DcMotor tapeExtend;

    public void onUpdate(Gamepad gamepad1, Gamepad gamepad2)
    {
             if (gamepad2.dpad_left)  { tapeExtend.setPower(-1.0); }
        else if (gamepad2.dpad_right) { tapeExtend.setPower( 1.0); }
        else                          { tapeExtend.setPower( 0.0); }
    }

    public MeasureTape(HardwareMap hardwareMap)
    {
        tapeExtend = hardwareMap.get(DcMotor.class, "tapeExtend");
        tapeAngle = hardwareMap.crservo.get("tapeAngle");

        tapeExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tapeExtend.setPower(0.0);

        tapeAngle.setPower(0.0);
    }
}
