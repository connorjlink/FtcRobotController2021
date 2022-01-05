package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
    This class abstracts away the management and initialization of all of the robot hardware onboard. This makes other opmodes fairly easy and without much duplication.
    To create an opmode using this hardware, the inherited class simply has to have a member variable of this class's type, and the constructor function for this class will be automatically called.
    This initialized the hardware map for each of the motors, servos, and usb device attached to the control hub
*/
public class RobotHardware
{
    //long list of robot hardware declarations
    public DcMotor frontLeftDrive = null, frontRightDrive = null, backLeftDrive = null, backRightDrive = null;

    public CRServo duckWheel = null;
    public DcMotor  intakeLifter = null, clawLifter = null;

    public CRServo tapeAngle = null;
    public DcMotor tapeExtend = null;

    public Servo intakeServo = null;
    public CRServo clawServo = null;

    public HardwareMap hardwareMap = null;
    public ElapsedTime elapsedTime = null;

    public static final double DRIVE_SPEED = 0.8;

    public static final double SERVO_LOCK = 0.5;

    public static final double INTAKE_STORE = 0.0;
    public static final double INTAKE_DUMP = 1.0;

    public static final double MAX_POWER = 1.0;
    public static final double HALF_POWER = 0.5;
    public static final double MIN_POWER = 0.0;

    public double ARM_TARGET = 0.0;

    public org.firstinspires.ftc.teamcode.PIDController pidArm;

    //if any toggle buttons are used, this function can be used to verify that the button switch didn't bounce and cause an erroneous state change event
    public boolean debounceOK()
    {
        return (elapsedTime.milliseconds() > 300);
    }


    public void setPowerLeft(double p)
    {
        frontLeftDrive.setPower(p);
        backLeftDrive.setPower(p);
    }

    public void setPowerRight(double p)
    {
        frontRightDrive.setPower(p);
        backRightDrive.setPower(p);
    }

    public void setPowerAll(double lf, double rf, double lb, double rb)
    {
        frontLeftDrive.setPower(lf);
        frontRightDrive.setPower(rf);

        backLeftDrive.setPower(lb);
        backRightDrive.setPower(rb);
    }

    public void setPowerAll(double p)
    {
        setPowerLeft(p);
        setPowerRight(p);
    }

    public void setModeAll(DcMotor.RunMode mode)
    {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }

    public void setBehaviorAll(DcMotor.ZeroPowerBehavior behavior)
    {
        frontLeftDrive.setZeroPowerBehavior(behavior);
        frontRightDrive.setZeroPowerBehavior(behavior);
        backLeftDrive.setZeroPowerBehavior(behavior);
        backRightDrive.setZeroPowerBehavior(behavior);
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
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        hardwareMap = hwMap;

        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");

        duckWheel = hardwareMap.crservo.get("duckWheel");
        intakeLifter = hardwareMap.dcMotor.get("intakeLifter");
        intakeServo = hardwareMap.servo.get("intakeServo");

        clawServo = hardwareMap.crservo.get("clawServo");
        clawLifter = hardwareMap.dcMotor.get("clawLifter");

        tapeAngle = hardwareMap.crservo.get("tapeAngle");
        tapeExtend = hardwareMap.dcMotor.get("tapeExtend");

        clawLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
            This if block checks whether or not the opmode is an autonomous program or not. For both our driver controlled programs and
            our autonomous programs, we select that they are not autonomous programs. This could be a useful feature if doing more complex paths
            such as bezier curves, but for our linear motion, we want to use a run to target operational mode for autonomous. Thus for both cases we can set
            the mode to RUN_WITHOUT_ENCODER, and then in the abstract autonomous initializer, we can STOP_AND_RESET_ENCODER, and then set
            RUN_TO_POSITION after specifying a target number of encoder counts we want each motor to move.
        */
        if (auto)
        {
            setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        else
        {
            setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        setPowerAll(0.0);

        //reverse the left motors of the robot since they are facing the opposite direction as the right motors
        //this makes setting the power in the opmodes much more intuitive, i.e. going forwards is (1,1,1,1) power for all motors
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        //verifies that all motor are on zero power. This should happen automatically, but no reason to chace having the robot move on initialization when it shouldn't
        duckWheel.setPower(0.0);
        intakeLifter.setPower(0.0);
        clawLifter.setPower(0.0);
        clawServo.setPower(0.0);
        intakeServo.setPosition(INTAKE_STORE);
    }

    //forwards through the constructor parameters to the initialization function above
    public RobotHardware(HardwareMap hardwareMap, boolean auto)
    {
        init(hardwareMap, auto);
    }
}