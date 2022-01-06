package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotHardware;

/*
    all autonomous programs we use will inherit this basic functionality--
    contains robot hardware intialization, an encoder drive function, and EasyOpenCV setup
    it uses the abstract class structure where no direct instance will be made of the parent class, only the children classes
*/
abstract public class AutonomousAbstract extends LinearOpMode
{
    //use a basic robot hardware
    public RobotHardware robot = null;
    //robot has a Logitech C310 camera
    public CameraOpenCV camera = null;
    //robot has an Intel RealSense T265 camera
    public CameraT265 slamra = null;
    //robot has an internal IMU in the Control Hub
    public IMU imu = null;

    //used for making accurate turns, stored as a member variable in case several functions need access to it in the future
    public org.firstinspires.ftc.teamcode.PIDController pidRotate;

    //andymark wheel & motor specs
    //private static final double COUNTS_PER_MOTOR_REV = 1120;
    //private static final double DRIVE_GEAR_REDUCTION = (15.0 / 20.0);
    //private static final double WHEEL_DIAMETER_INCHES = 4.0;

    //rev core hex motor specs
    private static final double CPMR_HEX = -288.0;
    private static final double DRIVE_GEAR_HEX = (30.0 / 60.0);
    private static final double COUNTS_PER_OUTPUT_REV = (CPMR_HEX / DRIVE_GEAR_HEX);
    private static final double COUNTS_PER_DEGREE = (360.0 / COUNTS_PER_OUTPUT_REV);


    //gobilda 5203 & gobilda mecanum specs
    private static final double COUNTS_PER_MOTOR_REV = 537.7;    //gobilda 5203 motor, andymark?
    private static final double DRIVE_GEAR_REDUCTION = 0.8333;     //for gobilda 1:1, for old robot ?
    private static final double WHEEL_DIAMETER_INCHES = 3.77953;     //gobilda 96mm
    private static final double COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /  (WHEEL_DIAMETER_INCHES * Math.PI));

    //will be used for the rev imu
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0.0;
    double rotation = 0.0;

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
        double degrees = degree;
        double power = robot.DRIVE_SPEED / 2;
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
        pidRotate.m_tolerance = 0.1;
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
        robot.setPowerAll(0.0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    //creates and initializes all the robot hardware, including the motors and camera
    public void onInit(String data)
    {
        //initializes all of the hardware onboard the robot
        robot = new RobotHardware(hardwareMap, false);

        pidRotate = new org.firstinspires.ftc.teamcode.PIDController(0.003, 0.000015, 0.00005);

        //zeroes out each motor's encoder
        robot.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //enables encoder use in the program
        robot.setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //to make the autonomous programs more reliable, set each motor to brake when they have no power applied
        robot.setBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE);

        //because motors are mounted backwards on the left side, reverse those motors
        robot.frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        camera = new CameraOpenCV("Webcam 1", data, hardwareMap);
        //slamra = new CameraT265(hardwareMap);
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
    angle is degrees, 0-180
     */
    public void setArm(double angle)
    {
        int counts = (int)(COUNTS_PER_DEGREE * angle);

        robot.clawLifter.setTargetPosition(counts);
        robot.clawLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.clawLifter.setPower(0.8);
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
        double angle = getAngle();
//
        org.firstinspires.ftc.teamcode.PIDController pidDrive =
                new org.firstinspires.ftc.teamcode.PIDController(0.005, 0.0001, 0.0002);
        pidDrive.setSetpoint(0.0);
        pidDrive.setOutputRange(0, robot.DRIVE_SPEED);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        int newLeftFrontTarget = 0, newRightFrontTarget = 0,
            newLeftBackTarget  = 0, newRightBackTarget  = 0;



        //verify that we won't crash the robot if internal data values are modified
        if (opModeIsActive())
        {
            robot.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newLeftFrontTarget  = robot.frontLeftDrive.getCurrentPosition()  + (int)(leftFrontInches  * COUNTS_PER_INCH);
            newRightFrontTarget = robot.frontRightDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget   = robot.backLeftDrive.getCurrentPosition()   + (int)(leftBackInches   * COUNTS_PER_INCH);
            newRightBackTarget  = robot.backRightDrive.getCurrentPosition()  + (int)(rightBackInches  * COUNTS_PER_INCH);

            //set up motor encoder drive targets, change their operating modes to run until they hit their targets, and start movement
            robot.frontLeftDrive.setTargetPosition(newLeftFrontTarget);
            robot.frontRightDrive.setTargetPosition(newRightFrontTarget);
            robot.backLeftDrive.setTargetPosition(newLeftBackTarget);
            robot.backRightDrive.setTargetPosition(newRightBackTarget);

            robot.setPowerAll(robot.DRIVE_SPEED / 2);


            robot.setModeAll(DcMotor.RunMode.RUN_TO_POSITION);



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
            double correction = pidDrive.performPID(getAngle() - angle);

            //set motors according to the received correction factor
            robot.setPowerLeft((robot.DRIVE_SPEED / 2) - correction);
            robot.setPowerRight((robot.DRIVE_SPEED / 2) + correction);

            //output internal encoder data to user in the opmode
            telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget,
                                                                                        newRightFrontTarget,
                                                                                        newLeftBackTarget,
                                                                                        newRightBackTarget);
            
            telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d", robot.frontLeftDrive.getCurrentPosition(),
                                                                                        robot.frontRightDrive.getCurrentPosition(),
                                                                                        robot.backLeftDrive.getCurrentPosition(),
                                                                                        robot.backRightDrive.getCurrentPosition());
            telemetry.update();
        }

        //stop each motor, since each's path is complete
        robot.setPowerAll(0.0);

        //reset encoder mode to the standard operating mode
        //robot.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        //small delay between instructions, gives robot time to stop
        //make smaller if need autonomous to go faster, longer if the robot is not stopping between each call of this function
        sleep(1000);
    }

    public void encoderDrive2(double inches)
    {
        double speed = robot.DRIVE_SPEED / 2.0;
        int newFLtarget, newFRtarget, newBLtarget, newBRtarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newFLtarget = robot.frontLeftDrive.getCurrentPosition()  + (int)(inches * COUNTS_PER_INCH);
            newFRtarget = robot.frontRightDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBLtarget = robot.backLeftDrive.getCurrentPosition()   + (int)(inches * COUNTS_PER_INCH);
            newBRtarget = robot.backRightDrive.getCurrentPosition()  + (int)(inches * COUNTS_PER_INCH);

            robot.frontLeftDrive.setTargetPosition(newFLtarget);
            robot.frontRightDrive.setTargetPosition(newFRtarget);
            robot.backLeftDrive.setTargetPosition(newBLtarget);
            robot.backRightDrive.setTargetPosition(newBRtarget);

            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                  (robot.frontLeftDrive.isBusy()  &&
                   robot.frontRightDrive.isBusy() &&
                   robot.backLeftDrive.isBusy()   &&
                   robot.backRightDrive.isBusy()))
            {
                telemetry.addData("Target",  "Running to %7d :%7d :%7d :%7d", newFLtarget, newFRtarget, newBLtarget, newBRtarget);

                telemetry.addData("At",  "Running at %7d :%7d", robot.frontLeftDrive.getCurrentPosition(),
                                                                robot.frontRightDrive.getCurrentPosition(),
                                                                robot.backLeftDrive.getCurrentPosition(),
                                                                robot.backRightDrive.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.setPowerAll(0.0);

            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(500);
        }
    }

    public void encoderDrive3(double inches)
    {
        double angle = getAngle();

        org.firstinspires.ftc.teamcode.PIDController pidDrive =
                new org.firstinspires.ftc.teamcode.PIDController(0.05, 0.0, 0.0);
        pidDrive.setSetpoint(0.0);
        pidDrive.setOutputRange(0, robot.DRIVE_SPEED);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        double speed = robot.DRIVE_SPEED / 2.0;
        int newFLtarget, newFRtarget, newBLtarget, newBRtarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newFLtarget = robot.frontLeftDrive.getCurrentPosition()  + (int)(inches * COUNTS_PER_INCH);
            newFRtarget = robot.frontRightDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBLtarget = robot.backLeftDrive.getCurrentPosition()   + (int)(inches * COUNTS_PER_INCH);
            newBRtarget = robot.backRightDrive.getCurrentPosition()  + (int)(inches * COUNTS_PER_INCH);




            robot.frontLeftDrive.setTargetPosition(newFLtarget);
            robot.frontRightDrive.setTargetPosition(newFRtarget);
            robot.backLeftDrive.setTargetPosition(newBLtarget);
            robot.backRightDrive.setTargetPosition(newBRtarget);

            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (robot.frontLeftDrive.isBusy()  &&
                     robot.frontRightDrive.isBusy() &&
                     robot.backLeftDrive.isBusy()   &&
                     robot.backRightDrive.isBusy()))
            {
                //obtain correction factor
                double correction = pidDrive.performPID(getAngle() - angle);

                robot.frontLeftDrive.setPower(speed - correction);
                robot.backLeftDrive.setPower(speed - correction);
                robot.frontRightDrive.setPower(speed + correction);
                robot.backRightDrive.setPower(speed + correction);

                telemetry.addData("Target",  "Running to %7d :%7d :%7d :%7d", newFLtarget, newFRtarget, newBLtarget, newBRtarget);

                telemetry.addData("At",  "Running at %7d :%7d", robot.frontLeftDrive.getCurrentPosition(),
                        robot.frontRightDrive.getCurrentPosition(),
                        robot.backLeftDrive.getCurrentPosition(),
                        robot.backRightDrive.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.setPowerAll(0.0);

            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(500);
        }
    }
}