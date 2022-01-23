package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class RobotV3
{
    private MecanumBot mecanumBot;
    private StaticIntake staticIntake;
    private DuckWheel duckWheel1, duckWheel2;
    private IMU imu;
    //add in the extra hardware later

    public void onUpdate(Gamepad gamepad1, Gamepad gamepad2)
    {
        mecanumBot.onUpdate(gamepad1, gamepad2);
        staticIntake.onUpdate(gamepad1, gamepad2);

        duckWheel1.onUpdate(gamepad1, gamepad2);
        duckWheel2.onUpdate(gamepad1, gamepad2);
    }

    public RobotV2(HardwareMap hardwareMap)
    {
        mecanumBot = new MecanumDrive(hardwareMap);
        staticIntake = new StaticIntake(hardwareMap);

        duckWheel1 = new DuckWheel(hardwareMap);
        duckWheel2 = new DuckWheel(hardwareMap);

        imu = new IMU(hardwareMap);
    }



    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose)
    {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed)
    {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading)
    {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose)
    {
        return new TrajectorySequenceBuilder
        (
            startPose,
            VEL_CONSTRAINT, ACCEL_CONSTRAINT,
            MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle)
    {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequenceBuilder(getPoseEstimate()).turn(angle).build());
    }

    public void turn(double angle)
    {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory)
    {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequenceBuilder(trajectory.start()).addTrajectory(trajectory).build());
    }

    public void followTrajectory(Trajectory trajectory)
    {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence)
    {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence)
    {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError()
    {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update()
    {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle()
    {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy()
    {
        return trajectorySequenceRunner.isBusy();
    }


    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3)
    {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) -imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}