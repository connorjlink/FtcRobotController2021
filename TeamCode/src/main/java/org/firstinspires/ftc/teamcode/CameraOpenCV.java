package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class CameraOpenCV
{
    public OpenCvWebcam webcam = null;
    public BlockDetector detector = null;

    public CameraOpenCV(String name, String autoID, HardwareMap hardwareMap)
    {
         /*
            This section does all of the Open CV initialization. It creates the camera object and its numeric alias,
            creates the pipeline with the detection class that we created, and open the camera stream on a new thread.
            TODO: implement error handling--not urgent
         */

        //find the webcam hardware device on the control hub, and obtain a handle to it
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, name), cameraMonitorViewId);

        //set the calculation pipeline: the duck/element position sensor we created is called for each frame of calculation when running the camera.
        //This is where we actually set up the camera to do the detection that we need it to
        detector = new BlockDetector(autoID);
        webcam.setPipeline(detector);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.

        /*
            Opens up the camera device on a new thread. The synchronous single-threaded function is now deprecated, so we have to make
            a lambda function that handles success and failure of the camera opening asynchronously. If the camera is successfully opened,
            the streaming starts, which begins to repeatedly process frames through the pipeline, which is where we detect ducks/elements.
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


    }
}