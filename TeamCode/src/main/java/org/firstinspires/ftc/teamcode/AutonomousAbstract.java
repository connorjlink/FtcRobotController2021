package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

/*
all autonomous programs we use will inherit this basic functionality--
    contains robot hardware intialization, an encoder drive function, and EasyOpenCV setup
it uses the abstract class structure where no direct instance will be made of the parent class, only the children classes
*/
abstract public class AutonomousAbstract extends LinearOpMode
{
    //use a basic robot hardware
    public RobotHardware robot = null;

    //hardware used for the EasyOpenCV block sensing system
    public OpenCvWebcam webcam;
    public BlockDetector detector = null;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime AutonomousTimeout = new ElapsedTime();

    //1120 or 2240 count possibly
    //calculates constants for encoder distance measurements in the autonomous

    //andydark cpr: ?
    //andymark gear : ?
    //andymark wdi : ?
    //andy mark stuff
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = (15.0 / 20.0);
    static final double WHEEL_DIAMETER_INCHES = 4.0;

    //for the arm--rev core hex motor
    static final double CPMR_HEX = -288.0;
    static final double DRIVE_GEAR_HEX = (30.0 / 60.0);
    static final double COUNTS_PER_OUTPUT_REV = (CPMR_HEX / DRIVE_GEAR_HEX);
    static final double COUNTS_PER_DEGREE = (360.0 / COUNTS_PER_OUTPUT_REV);


    //gobilda stuff
   // static final double COUNTS_PER_MOTOR_REV = 537.7;    //gobilda 5203 motor, andymark?
   // static final double DRIVE_GEAR_REDUCTION = 1.0;     //for gobilda 1:1, for old robot ?
   // static final double WHEEL_DIAMETER_INCHES = 3.77953;     //gobilda 96mm
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /  (WHEEL_DIAMETER_INCHES * Math.PI);

    //will be used for the rev imu
    private double previousHeading = 0;
    private double integratedHeading = 0;
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0.0;
    double rotation = 0.0;

    //autonomous program specific hardware on the robot: control hub imu, and an arm controller
    public BNO055IMU imu = null;

    /*
     * This method returns a value of the Z axis of the REV Expansion Hub IMU.
     * It transforms the value from (-180, 180) to (-inf, inf).
     * This code was taken and modified from
     */
    //sourced from https://stemrobotics.cs.pdx.edu/node/7265
    //zeroes out the measured angle from the imu
    protected void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    //sourced from https://stemrobotics.cs.pdx.edu/node/7265
    //modified and inspired from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481
    //takes IMU (-180,180) angle range and converts it to (-inf,inf)
    //obtains the angle that the robot is on
    protected double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /*
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    //from  https://stemrobotics.cs.pdx.edu/node/7265
    protected void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int)Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        org.firstinspires.ftc.teamcode.PIDController pidRotate =
                new org.firstinspires.ftc.teamcode.PIDController(0.003, 0.00003, 0.0);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                robot.frontLeftDrive.setPower(power);
                robot.backLeftDrive.setPower(power);

                robot.frontRightDrive.setPower(-power);
                robot.backRightDrive.setPower(-power);

                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.frontLeftDrive.setPower(-power);
                robot.backLeftDrive.setPower(-power);

                robot.frontRightDrive.setPower(power);
                robot.backRightDrive.setPower(power);

            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.frontLeftDrive.setPower(-power);
                robot.backLeftDrive.setPower(-power);

                robot.frontRightDrive.setPower(power);
                robot.backRightDrive.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.frontLeftDrive.setPower(0.0);
        robot.backLeftDrive.setPower(0.0);
        robot.frontRightDrive.setPower(0.0);
        robot.backRightDrive.setPower(0.0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    //creates and initializes all the robot hardware, including the motors and camera
    public void onInit(String data)
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();



        //initializes all of the hardware onboard the robot
        robot = new RobotHardware(hardwareMap, false);

        //zeroes out each motor's encoder
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //enables encoder use in the program
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //to make the autonomous programs more reliable, set each motor to brake when they have no power applied
        robot.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //because motors are mounted backwards on the left side, reverse those motors
        robot.backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);


        /*
        This section does all of the Open CV initialization. It creates the camera object and its numeric alias,
        creates the pipeline with the detection class that we created, and open the camera stream on a new thread.
        TODO: implement error handling
         */

        //find the webcam hardware device on the control hub, and obtain a handle to it
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //set the calculation pipeline: the duck/element position sensor we created is called for each frame of calculation when running the camera.
        //This is where we actually set up the camera to do the detection that we need it to
        detector = new BlockDetector(data);
        webcam.setPipeline(detector);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        /*
        Opens up the camera device on a new thread. The synchronous single-threaded function is now deprecated, so we have to make
        a lambda function that handles success and failure of the camera opening.
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        //AutonomousTimeout = new ElapsedTime();
        //AutonomousTimeout.reset();

        waitForStart();
    }

    /*
    angle is degrees, 0-180
     */
    public void setArm(double angle)
    {
        int counts = (int)(COUNTS_PER_DEGREE * angle);

        robot.clawLifter.setTargetPosition(counts);
        robot.clawLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.clawLifter.setPower(0.8);
    }

    //set each motor to the designated power for the designated amount of time, then stops them
    //currently usused
    public void timedDrive(double fl, double fr, double bl, double br, long time)
    {
        robot.frontLeftDrive.setPower(fl);
        robot.frontRightDrive.setPower(fr);
        robot.backLeftDrive.setPower(bl);
        robot.backRightDrive.setPower(br);

        sleep(time);

        robot.frontLeftDrive.setPower(robot.MIN_POWER);
        robot.frontRightDrive.setPower(robot.MIN_POWER);
        robot.backLeftDrive.setPower(robot.MIN_POWER);
        robot.backRightDrive.setPower(robot.MIN_POWER);
    }

    /*
    Encoder powered drive function. Obtains how far each wheel needs to travel, and sets up the motor to run until they all reach those positions.
    maybe TODO: make more generic and usable for robot arm
     */
    public void encoderDrive(double leftFrontInches,
                             double rightFrontInches,
                             double leftBackInches,
                             double rightBackInches)
    {
        //PIDFController pi
        //dDrive = new PIDFController(0.0,
        double correction = 0.0;
        double angle = getAngle();

        double speed = robot.DRIVE_SPEED;

        org.firstinspires.ftc.teamcode.PIDController pidDrive =
                new org.firstinspires.ftc.teamcode.PIDController(0.05, 0.0, 0.0);
        pidDrive.setSetpoint(0.0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;

        //verify that we won't crash the robot if internal data values are modified
        if (opModeIsActive())
        {
            newLeftFrontTarget  = robot.frontLeftDrive.getCurrentPosition()  + (int)(leftFrontInches  * COUNTS_PER_INCH);
            newRightFrontTarget = robot.frontRightDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget   = robot.backLeftDrive.getCurrentPosition()   + (int)(leftBackInches   * COUNTS_PER_INCH);
            newRightBackTarget  = robot.backRightDrive.getCurrentPosition()  + (int)(rightBackInches  * COUNTS_PER_INCH);

            //set up motor encoder drive targets, change their operating modes to run until they hit their targets, and start movement
            robot.frontLeftDrive.setTargetPosition(newLeftFrontTarget);
            robot.frontRightDrive.setTargetPosition(newRightFrontTarget);
            robot.backLeftDrive.setTargetPosition(newLeftBackTarget);
            robot.backRightDrive.setTargetPosition(newRightBackTarget);

            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //use isBusy || isBusy if all motors need to reach their targets
        //using this mode can cause bugs relating to over turning targets inconsistently
        while (opModeIsActive() &&
                (robot.frontLeftDrive.isBusy()  &&
                 robot.frontRightDrive.isBusy() &&
                 robot.backLeftDrive.isBusy()   &&
                 robot.backRightDrive.isBusy()))
        {
            //obtain correction factor
            correction = pidDrive.performPID(getAngle() - angle);

            //set motors according to the received correction factor
            robot.frontLeftDrive.setPower(speed - correction);
            robot.backLeftDrive.setPower(speed - correction);

            robot.frontRightDrive.setPower(speed + correction);
            robot.backRightDrive.setPower(speed + correction);

            //output internal encoder data to user in the opmode
            telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
            telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d", robot.frontLeftDrive.getCurrentPosition(),
                    robot.frontRightDrive.getCurrentPosition(),
                    robot.backLeftDrive.getCurrentPosition(),
                    robot.backRightDrive.getCurrentPosition());
            telemetry.update();
        }

        //stop each motor, since each's path is complete
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);

        //reset encoder mode to the standard operating mode
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //small delay between instructions, gives robot time to stop
        //make smaller if need autonomous to go faster, longer if the robot is not stopping between each call of this function
        sleep(400);
    }
}