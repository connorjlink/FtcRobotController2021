package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

/*
This class abstracts away the management and initialization of all of the robot hardware onboard. This makes other opmodes fairly easy and without much duplication.
To create an opmode using this hardware, the inherited class simply has to have a member variable of this class's type, and the constructor function for this class will be automatically called.
This initialized the hardware map for each of the motors, servos, and usb device attached to the control hub
 */
public class RobotHardware
{
    //long list of robot hardware declarations
    public DcMotor frontLeftDrive = null, frontRightDrive = null, backLeftDrive = null, backRightDrive = null;
    public DcMotor duckWheel = null,  intakeLifter = null, clawLifter = null;

    public Servo intakeServo = null;
    public CRServo clawServo = null;

    public HardwareMap hardwareMap = null;
    public ElapsedTime elapsedTime = null;

    public final double COUNTS_PER_INCH = 735.92113;
    public double DRIVE_SPEED = 0.65;

    public final double SERVO_LOCK = 0.5;

    public final double INTAKE_STORE = 0.0;
    public final double INTAKE_DUMP = 1.0;

    public final double MAX_POWER = 1.0;
    public final double HALF_POWER = 0.5;
    public final double MIN_POWER = 0.0;

    public double ARM_TARGET = 0.0;

    public org.firstinspires.ftc.teamcode.PIDController pidArm;

    public Transform2d cameraToRobot = null;
    public final double encoderMeasurementCovariance = 0.8;
    public Pose2d startingPose = null;
    public T265Camera slamra = null;

    //if any toggle buttons are used, this function can be used to verify that the button switch didn't bounce and cause an erroneous state change event
    public boolean debounceOK()
    {
        return (elapsedTime.milliseconds() > 300);
    }

    public void setPowerAll(double lf, double rf, double rb, double lb)
    {
        frontLeftDrive.setPower(lf);
        frontRightDrive.setPower(rf);
        backLeftDrive.setPower(rb);
        backRightDrive.setPower(lb);
    }

    /*
    In this function everything is initialized. As a parameter this function takes in a reference to the current hardware map that is part of the opmode.
    Without this, we wouldn't be able to initialize the hardware properly.

    For initialization:
        -creates the hardware devices on the control hub
        -sets the correct operational mode for each device
        -verifies everything is set to a power level of zero
     */
    public void init(HardwareMap hwMap, boolean auto)
    {
        //pidArm = new org.firstinspires.ftc.teamcode.PIDController(0.05, 0.0005, 0.001);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        hardwareMap = hwMap;

        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");

        duckWheel = hardwareMap.dcMotor.get("duckWheel");
        intakeLifter = hardwareMap.dcMotor.get("intakeLifter");
        intakeServo = hardwareMap.servo.get("intakeServo");
        clawServo = hardwareMap.crservo.get("clawServo");
        clawLifter = hardwareMap.dcMotor.get("clawLifter");

        clawLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        This if block checks whether or not the opmode is an autonomous program or not. For both our driver controlled programs and
        our autonomous programs, we select that they are not autonomous programs. This could be a useful feature if doing more complex paths
        such as bezier curves, but for our linear motion, we want to use a run to target operational mode for autonomous. Thus for both cases we can set
        the mode to RUN_WITHOUT_ENCODER, and then in the abstract autonomous initializer, we can STOP_AND_RESET_ENCODER, and then set
        RUN_TO_POSITION after specifying a target number of encoder counts we want each motor to move.
         */
        if (auto)
        {
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        else
        {
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        frontLeftDrive.setPower(MIN_POWER);
        frontRightDrive.setPower(MIN_POWER);
        backLeftDrive.setPower(MIN_POWER);
        backRightDrive.setPower(MIN_POWER);

        //reverse the left motors of the robot since they are facing the opposite direction as the right motors
        //this makes setting the power in the opmodes much more intuitive, i.e. going forwards is (1,1,1,1) power for all motors
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        //verifies that all motor are on zero power. This should happen automatically, but no reason to chace having the robot move on initialization when it shouldn't
        duckWheel.setPower(MIN_POWER);
        intakeLifter.setPower(MIN_POWER);
        clawLifter.setPower(MIN_POWER);
        intakeServo.setPosition(INTAKE_STORE);
        clawServo.setPower(MIN_POWER);

        //initialize camera stuffs
        cameraToRobot = new Transform2d();
        startingPose = new Pose2d(1, 1, new Rotation2d());
        slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        slamra.setPose(startingPose);
        slamra.start();
    }

    //forwards through the constructor parameters to the initialization function above
    public RobotHardware(HardwareMap hardwareMap, boolean auto)
    {
        init(hardwareMap, auto);
    }
}