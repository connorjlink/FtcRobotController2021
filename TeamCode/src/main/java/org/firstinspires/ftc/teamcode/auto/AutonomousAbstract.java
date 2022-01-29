package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.component.CameraOpenCV;
import org.firstinspires.ftc.teamcode.component.IMU;
import org.firstinspires.ftc.teamcode.component.RobotV2;

import org.firstinspires.ftc.teamcode.helper.PIDController;

/*
    all autonomous programs we use will inherit this basic functionality--
    contains robot hardware intialization, an encoder drive function, and EasyOpenCV setup
    it uses the abstract class structure where no direct instance will be made of the parent class, only the children classes
*/
abstract public class AutonomousAbstract extends LinearOpMode
{
    //use a basic robot hardware
    public RobotV2 robot = null;
    //robot has a Logitech C310 camera
    public CameraOpenCV camera = null;
    //robot has an internal IMU in the Control Hub
    public IMU imu = null;

    //used for making accurate turns, stored as a member variable in case several functions need access to it in the future
    public PIDController pidRotate;

    //andymark wheel & motor specs
    //private static final double COUNTS_PER_MOTOR_REV = 1120;
    //private static final double DRIVE_GEAR_REDUCTION = (15.0 / 20.0);
    //private static final double WHEEL_DIAMETER_INCHES = 4.0;

    //gobilda 5203 & gobilda mecanum specs
    private static final double COUNTS_PER_MOTOR_REV = 537.7;    //gobilda 5203 motor, andymark?
    private static final double DRIVE_GEAR_REDUCTION = 0.9375;     //for gobilda 1:1, for old robot ?
    private static final double WHEEL_DIAMETER_INCHES = 3.77953;     //gobilda 96mm
    private static final double COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /  (WHEEL_DIAMETER_INCHES * Math.PI));

    public static final double INTAKE_STORE = 0.0;
    public static final double INTAKE_DUMP = 1.0;

    //will be used for the rev imu
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0.0;
    double rotation = 0.0;

    public void setLifter(int counts)
    {
        //robot.intakeLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.blockLifter.blockLifter.setTargetPosition(counts);
        robot.blockLifter.blockLifter.setPower(0.8);
        robot.blockLifter.blockLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.blockLifter.blockLifter.isBusy()) ;
        robot.blockLifter.blockLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.blockLifter.blockLifter.setPower(0.0);
    }

    public void setArm(int counts)
    {
        robot.clawLifter.clawLifter.setPower(0.0);
        robot.clawLifter.clawLifter.setTargetPosition(counts);

        if (counts < 50) robot.clawLifter.clawLifter.setPower(0.65);
        else             robot.clawLifter.clawLifter.setPower(0.3);

        robot.clawLifter.clawLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.clawLifter.clawLifter.isBusy()) ;
        robot.clawLifter.clawLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.clawLifter.clawLifter.setPower(0.0);
    }

    /*
     * This method returns a value of the Z axis of the REV Expansion Hub IMU.
     * It transforms the value from (-180, 180) to (-inf, inf).
     * This code was taken and modified from
     */
    //sourced from https://stemrobotics.cs.pdx.edu/node/7265
    //zeroes out the measured angle from the imu
    protected void resetAngle()
    {
        lastAngles = imu.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

        /*
            We want to measure error in degrees later on, so take degrees from the IMU.
            For axes ordering, we want to use intrinsic angles. This is due to the nature of how the system works.
            Instrinsic angles are defined as rotations relative to the already transformed state of something.
            Extrinsic angles are defined as rotations relative to the global coordinate system.
            Because instrinsic angles are inherently reliant upon each other, we can simply use the a different
            axis swizzling to avoid having to use Euler Angles/Spherical projection manually.s
        */

        Orientation angles = imu.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
    protected void rotate(int degree)
    {
        double degrees = -degree;
        double power = 0.55;
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.m_tolerance = 1;
        //pidRotate.setTolerance(0.1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                robot.mecanumBot.frontLeftDrive.setPower(power);
                robot.mecanumBot.backLeftDrive.setPower(power);

                robot.mecanumBot.frontRightDrive.setPower(-power);
                robot.mecanumBot.backRightDrive.setPower(-power);

                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.mecanumBot.frontLeftDrive.setPower(-power);
                robot.mecanumBot.backLeftDrive.setPower(-power);

                robot.mecanumBot.frontRightDrive.setPower(power);
                robot.mecanumBot.backRightDrive.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.mecanumBot.frontLeftDrive.setPower(-power);
                robot.mecanumBot.backLeftDrive.setPower(-power);

                robot.mecanumBot.frontRightDrive.setPower(power);
                robot.mecanumBot.backRightDrive.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.mecanumBot.setPower(0.0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(250);

        // reset angle tracking on new heading.
        resetAngle();
    }

    //creates and initializes all the robot hardware, including the motors and camera
    public void onInit(String data)
    {
        //initializes all of the hardware onboard the robot
        robot = new RobotV2(hardwareMap);

        pidRotate = new PIDController(0.003, 0.000015, 0.00005);

        //zeroes out each motor's encoder
        robot.mecanumBot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //enables encoder use in the program
        robot.mecanumBot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //fixes arm
        robot.clawLifter.clawLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.clawLifter.clawLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //to make the autonomous programs more reliable, set each motor to brake when they have no power applied
        robot.mecanumBot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //because motors are mounted backwards on the left side, reverse those motors
        robot.mecanumBot.frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.mecanumBot.backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        camera = new CameraOpenCV("Webcam 1", data, hardwareMap);
        imu = new IMU("imu", hardwareMap);

        //wait for the IMU to calibrate before proceeding
        while (!imu.getIMU().isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addLine("IMU Calibrated");
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
    }

    /*
        Encoder powered drive function. Obtains how far each wheel needs to travel, and sets up the motor to run until they all reach those positions.
        maybe TODO: make more generic and usable for robot arm
    */
    public void encoderDrive(double leftFrontInches,
                             double rightFrontInches,
                             double leftBackInches,
                             double rightBackInches, double timeout)
    {
        ElapsedTime driveTime = new ElapsedTime();
        driveTime.reset();

        double angle = getAngle();
//
        PIDController pidDrive = new PIDController(0.005, 0.0001, 0.0002);

        pidDrive.setSetpoint(0.0);
        pidDrive.setOutputRange(0, 0.8);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        int newLeftFrontTarget = 0, newRightFrontTarget = 0,
            newLeftBackTarget  = 0, newRightBackTarget  = 0;

        double power = 0.5;

        //verify that we won't crash the robot if internal data values are modified
        if (opModeIsActive())
        {
            robot.mecanumBot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newLeftFrontTarget  = robot.mecanumBot.frontLeftDrive.getCurrentPosition()  + (int)(leftFrontInches  * COUNTS_PER_INCH);
            newRightFrontTarget = robot.mecanumBot.frontRightDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget   = robot.mecanumBot.backLeftDrive.getCurrentPosition()   + (int)(leftBackInches   * COUNTS_PER_INCH);
            newRightBackTarget  = robot.mecanumBot.backRightDrive.getCurrentPosition()  + (int)(rightBackInches  * COUNTS_PER_INCH);

            //set up motor encoder drive targets, change their operating modes to run until they hit their targets, and start movement
            robot.mecanumBot.frontLeftDrive.setTargetPosition(newLeftFrontTarget);
            robot.mecanumBot.frontRightDrive.setTargetPosition(newRightFrontTarget);
            robot.mecanumBot.backLeftDrive.setTargetPosition(newLeftBackTarget);
            robot.mecanumBot.backRightDrive.setTargetPosition(newRightBackTarget);

            robot.mecanumBot.setPower(power);
            robot.mecanumBot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //use isBusy || isBusy if all motors need to reach their targets
        //using this mode can cause bugs relating to over turning targets inconsistently
        while (opModeIsActive()                          &&
              (robot.mecanumBot.frontLeftDrive.isBusy()  &&
               robot.mecanumBot.frontRightDrive.isBusy() &&
               robot.mecanumBot.backLeftDrive.isBusy()   &&
               robot.mecanumBot.backRightDrive.isBusy()  &&
                driveTime.seconds() < timeout))
        {
            //obtain correction factor
            double correction = pidDrive.performPID(getAngle() - angle);

            //set motors according to the received correction factor
            robot.mecanumBot.setPowerLeft(power - correction);
            robot.mecanumBot.setPowerRight(power + correction);

            //output internal encoder data to user in the opmode
            telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget,
                                                                        newRightFrontTarget,
                                                                        newLeftBackTarget,
                                                                        newRightBackTarget);
            
            telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d", robot.mecanumBot.frontLeftDrive.getCurrentPosition(),
                                                                                        robot.mecanumBot.frontRightDrive.getCurrentPosition(),
                                                                                        robot.mecanumBot.backLeftDrive.getCurrentPosition(),
                                                                                        robot.mecanumBot.backRightDrive.getCurrentPosition());
            telemetry.update();
        }

        //stop each motor, since each's path is complete
        robot.mecanumBot.setPower(0.0);

        //reset encoder mode to the standard operating mode
        robot.mecanumBot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //small delay between instructions, gives robot time to stop
        //make smaller if need autonomous to go faster, longer if the robot is not stopping between each call of this function
        sleep(1000);
    }
}