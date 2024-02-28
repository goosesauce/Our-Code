package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Calibration Colours", group = "Robot")
public class Calibration extends OpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here

        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(960, 544, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("working", "");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed", "");
                telemetry.update();
            }

        });
    }

    @Override
    public void loop() {
        telemetry.addData("Zone 1 Red:", pipeline.zoneRedPercentage[0]);
        telemetry.addData("Zone 2 Red:", pipeline.zoneRedPercentage[1]);
        telemetry.addData("Zone 3 Red:", pipeline.zoneRedPercentage[2]);

        // Output min and max HSV values to telemetry
        telemetry.addData("Min HSV: ", pipeline.minHSV.toString());
        telemetry.addData("Max HSV: ", pipeline.maxHSV.toString());

        telemetry.update();
    }
}

class SamplePipeline extends OpenCvPipeline {
    Mat hsvImage = new Mat();
    int[] zoneRedPercentage = new int[3];
    public Scalar minHSV = new Scalar(255, 255, 255); // Initialize with maximum HSV values
    public Scalar maxHSV = new Scalar(0, 0, 0); // Initialize with minimum HSV values

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input frame to HSV color space
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

        // Define the lower and upper bounds for the red color in HSV space
        Scalar lowerRed = new Scalar(0, 100, 100); // Adjust these values as needed
        Scalar upperRed = new Scalar(20, 255, 255); // Adjust these values as needed

        // Create a binary mask to isolate the red color
        Mat mask = new Mat();
        Core.inRange(hsvImage, lowerRed, upperRed, mask);

        // Define the regions of interest (ROIs) for the three zones
        Rect roi1 = new Rect(270, 230, 80, 80);

        // Draw rectangles to visualize the three zones
        Imgproc.rectangle(input, roi1, new Scalar(0, 0, 255), 2);

        processROI(input, roi1, "test");

        // Release Mat objects to prevent memory leaks
        mask.release();

        return input;
    }

    private void processROI(Mat input, Rect roi, String label) {
        Mat roiImage = new Mat(hsvImage, roi);

        Scalar roiMinHSV = new Scalar(255, 255, 255); // Initialize with maximum HSV values
        Scalar roiMaxHSV = new Scalar(0, 0, 0); // Initialize with minimum HSV values

        for (int y = 0; y < roiImage.rows(); y++) {
            for (int x = 0; x < roiImage.cols(); x++) {
                double[] pixel = roiImage.get(y, x);

                if (pixel != null && pixel.length == 3) {
                    roiMinHSV.val[0] = Math.min(roiMinHSV.val[0], pixel[0]); // Hue
                    roiMinHSV.val[1] = Math.min(roiMinHSV.val[1], pixel[1]); // Saturation
                    roiMinHSV.val[2] = Math.min(roiMinHSV.val[2], pixel[2]); // Value

                    roiMaxHSV.val[0] = Math.max(roiMaxHSV.val[0], pixel[0]);
                    roiMaxHSV.val[1] = Math.max(roiMaxHSV.val[1], pixel[1]);
                    roiMaxHSV.val[2] = Math.max(roiMaxHSV.val[2], pixel[2]);
                }
            }
        }

        minHSV = roiMinHSV;
        maxHSV = roiMaxHSV;
    }
}
