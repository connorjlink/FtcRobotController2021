package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotV2
{
    public MecanumBot mecanumBot;
    public ClawLifter clawLifter;
    public BlockLifter blockLifter;
    public DuckWheel duckWheel;
    public MeasureTape measureTape;

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
        mecanumBot = new MecanumBot(hardwareMap);
        clawLifter = new ClawLifter(hardwareMap);
        blockLifter = new BlockLifter(hardwareMap);
        duckWheel = new DuckWheel(hardwareMap);
        measureTape = new MeasureTape(hardwareMap);
    }
}