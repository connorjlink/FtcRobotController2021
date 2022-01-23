package org.firstinspires.ftc.teamcode;

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
             if (gamepad2.dpad_left)  { robot.tapeExtend.setPower(-1.0); }
        else if (gamepad2.dpad_right) { robot.tapeExtend.setPower( 1.0); }
        else                          { robot.tapeExtend.setPower( 0.0); }
    }

    public StaticIntake(HardwareMap hardwareMap)
    {
        tapeExtend = hardwareMap.dcmotor.get("tapeExtend");
        tapeAngle = hardwareMap.crservo.get("tapeAngle");

        tapeExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tapeExtend.setPower(0.0);

        tapeAngle.setPower(0.0);
    }
}
