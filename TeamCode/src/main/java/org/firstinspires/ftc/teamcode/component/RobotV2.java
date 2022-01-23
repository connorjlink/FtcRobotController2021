package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class RobotV2
{
    private MecanumBot mecanumBot;
    private ClawLifter clawLifter;
    private BlockLifter blockLifter;
    private DuckWheel duckWheel;
    private MeasureTape measureTape;

    public void onUpdate(Gamepad gamepad1, Gamepad gamepad2)
    {
        mecanumBot.onUpdate(gamepad1, gamepad2);
        clawLifter.onUpdate(gamepad1, gamepad2);
        blockLifter.onUpdate(gamepad1, gamepad2);
        duckWheel.onUpdate(gamepad1, gamepad2);
        measureTape.onUpdate(gamepad1, gamepad2);
    }

    public RobotV2(HardwareMap hardwareMap)
    {
        mecanumBot = new MecanumDrive(hardwareMap);
        clawLifter = new ClawLifter(hardwareMap);
        blockLifter = new BlockLifter(hardwareMap);
        duckWheel = new DuckWheel(hardwareMap);
        measureTape = new MeasureTape(hardwareMap);
    }
}