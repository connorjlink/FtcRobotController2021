package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

public class IMU
{
    private BNO055IMU imu = null;

    public IMU(String name)
    {
        //set up the IMU's method of measurement
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //create the IMU device and initalize it
        imu = hardwareMap.get(BNO055IMU.class, name);
        imu.initialize(parameters);

        //wait for the IMU to calibrate before proceeding
        while (!imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
    }
}