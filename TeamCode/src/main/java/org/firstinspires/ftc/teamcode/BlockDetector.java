package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/*
This class represents an essentially "singleton", or single instance of a block detection unit. This class will be used in the pipeline of the camera
stream for the autonomous. It takes in raw frame data from OpenCV and attempts to determine on which tape piece the duck/capstone is located
 */
public class BlockDetector extends OpenCvPipeline
{
    private Mat workingMatrix = new Mat();
    public String data = null;
    public String name = null;
    public double right = 0.0, left = 0.0;

    //ctor, takes in name from each autonomous program class, as for Red B position, the robot is aligned differently relative to the tape pieces
    public BlockDetector(String auto)
    {
        name = auto;
    }

    @Override
    //implements the abstract function from the parent class; is the function that does the detecting work
    public final Mat processFrame(Mat input)
    {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty())
        {
            return input;
        }

        /*
        Convert the image RGB colorspace to YCbCr colorspace. This isn't technically necessary, but makes detecting yellow very easy.
        This is because in this colorspace:
            Y = the pixel luminance or brightness component
            Cb = blue difference, measures the difference between the amount of blue and the amount of inverse blue present
            Cr = red difference, measures the difference between the amount of red and the amount of inverse red present

         For our use, we can measure the Cb component, and when it is less, this means that there is more yellow present
         */
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        //obtain the two portions of the screen that we are interested in: when the robot is lined up properly, this will be two rectangles that surround two of the tape pieces
        Mat matLeft = workingMatrix.submat(80, 200, 10, 110);
        Mat matRight = workingMatrix.submat(80, 200, 190, 290);

        //highlight the rectangles that we obtain matrices to on the camera stream output on the driver station app
        Imgproc.rectangle(workingMatrix,
                new Rect(10, 80, 100, 120),
                new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix,
                new Rect(190, 80, 100, 120),
                new Scalar(0, 255, 0));

        //obtain the total amount of Cb in each rectangle
        //when comparing totals, less amounts of Cb means more yellow present
        double leftTotal = Core.sumElems(matLeft).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];

        right = rightTotal;
        left = leftTotal;

        //in all positions except for Red B, the robot is aligned to view on the camera the rightmost 2 pieces of tape
        //less Cb total means more yellow present
        //since the camera can only see two of elements, we can use the difference between the other two in order to determine which position the object in
        //if the two viewed positions have very similar Cb totals, then the block must be in the non-viewed position
        if (name != "redB")
        {
            if (leftTotal > rightTotal)
            {
                data = "r";
            }

            else
            {
                data = "c";
                //data = "left";
            }

            //100,000 is the currently used Cb delta, if rectangles are reframed, then this number will need to be scaled, since it's an absolute quantity rather than a relative one
            if (Math.abs(rightTotal - leftTotal) < 100000.0)
            {
                data = "l";
            }
        }

        //in Red B autonomous, the robot is aligned to the left 2 pieces of tape, so detection is generally the same, just with the positions of each comparison being different
        /*
        
        */

        else
        {
            if (leftTotal > rightTotal)
            {
                data = "c";
            }

            else
            {
                data = "l";
            }

            if (Math.abs(rightTotal - leftTotal) < 100000.0)
            {
                data = "r";
            }
        }

        return workingMatrix;
    }
}
