package org.firstinspires.ftc.teamcode;

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
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /  (WHEEL_DIAMETER_INCHES * 3.1415926535);

    //will be used for the rev imu
    private double previousHeading = 0;
    private double integratedHeading = 0;
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0.0;

    //autonomous program specific hardware on the robot: control hub imu, and an arm controller
    public BNO055IMU imu = null;
    public PIDController pidRotate = null;

    /*
     * This method returns a value of the Z axis of the REV Expansion Hub IMU.
     * It transforms the value from (-180, 180) to (-inf, inf).
     * This code was taken and modified from
     */
    //sourced from https://stemrobotics.cs.pdx.edu/node/7265
    //zeroes out the measured angle from the imu
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    //sourced from https://stemrobotics.cs.pdx.edu/node/7265
    //modified and inspired from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481
    //takes IMU (-180,180) angle range and converts it to (-inf,inf)
    //obtains the angle that the robot is on
    private double getAngle()
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
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        robot.setPowerAll(leftPower, rightPower, leftPower, rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        robot.setPowerAll(0.0, 0.0, 0.0, 0.0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    //creates and initializes all the robot hardware, including the motors and camera
    public void onInit(String data)
    {
        //AutonomousTimeout = new ElapsedTime();
        //AutonomousTimeout.reset();

        /*
        //configures the REV Control Hub Inertial Measurement Unit (IMU)
        //this is a built in gyroscope that we can use to make turns more accurate
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        //creates the IMU device in the hardware map and sets up the configuration oboard that we created
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //verifies that the IMU is calibrated before doing anything else
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }*/

        //thi s an slite difrence
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
    Encoder powered arm function. Obtains how far the arm needs to travel, and sets up the motor to run until it reaches that position
     */
    public void encoderMotor(double speed,
                             double armInches)
    {

    }


    /*
    Encoder powered drive function. Obtains how far each wheel needs to travel, and sets up the motor to run until they all reach those positions.
    maybe TODO: make more generic and useable for robot arm
     */
    public void encoderDrive(double speed,
                             double leftFrontInches,
                             double rightFrontInches,
                             double leftBackInches,
                             double rightBackInches, double unused)
    {
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;

        //verify that we won't crash the robot if internal data values are modified
        if (opModeIsActive())
        {
            newLeftFrontTarget = robot.frontLeftDrive.getCurrentPosition()   + (int)(leftFrontInches  * COUNTS_PER_INCH);
            newRightFrontTarget = robot.frontRightDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.backLeftDrive.getCurrentPosition()     + (int)(leftBackInches   * COUNTS_PER_INCH);
            newRightBackTarget = robot.backRightDrive.getCurrentPosition()   + (int)(rightBackInches  * COUNTS_PER_INCH);

            //set up motor encoder drive targets, change their operating modes to run until they hit their targets, and start movement
            robot.frontLeftDrive.setTargetPosition(newLeftFrontTarget);
            robot.frontRightDrive.setTargetPosition(newRightFrontTarget);
            robot.backLeftDrive.setTargetPosition(newLeftBackTarget);
            robot.backRightDrive.setTargetPosition(newRightBackTarget);

            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.frontLeftDrive.setPower(speed);
            robot.frontRightDrive.setPower(speed);
            robot.backLeftDrive.setPower(speed);
            robot.backRightDrive.setPower(speed);
        }

        //use isBusy || isBusy if all motors need to reach their targets
        //using this mode can cause bugs relating to over turning targets inconsistently
        while (opModeIsActive() &&
                (robot.frontLeftDrive.isBusy()  &&
                 robot.frontRightDrive.isBusy() &&
                 robot.backLeftDrive.isBusy()   &&
                 robot.backRightDrive.isBusy()))
        {

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
        sleep(500);
    }
}