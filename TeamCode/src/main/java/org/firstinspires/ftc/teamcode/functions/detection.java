package org.firstinspires.ftc.teamcode.functions;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class detection extends OpenCvPipeline {
    public static int[] zoneColourPercentage = new int[3];
    static Mat hsvImage = new Mat();
    static Mat mask = new Mat();
    public static String allianceColour = "";

    static Rect roi1 = new Rect(230, 410, 80, 80);
    static Rect roi2 = new Rect(645, 380, 80, 80);
    static Rect roi3 = new Rect(1150, 410, 80, 80);
    //test

    @Override
    public Mat processFrame(Mat input) {
        try {
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);


            Scalar lowerRed = new Scalar(0, 100, 100);
        Scalar upperRed = new Scalar(179, 255, 255);

        Scalar lowerBlue = new Scalar(90, 100, 100);
        Scalar upperBlue = new Scalar(130, 255, 255);



        switch (allianceColour) {
            case "Blue":
                Core.inRange(hsvImage, lowerBlue, upperBlue, mask);
                break;

            case "Red":
                Core.inRange(hsvImage, lowerRed, upperRed, mask);
                break;
        }

        Imgproc.line(input, new Point(0, 670), new Point(560, 420), new Scalar(0, 255, 255), 2);
        Imgproc.line(input, new Point(820, 420), new Point(1280, 580), new Scalar(0, 255, 255), 2);
        Imgproc.line(input, new Point(0, 490), new Point(1280, 490), new Scalar(255, 0, 255), 2);

        Imgproc.rectangle(input, roi1, new Scalar(0, 0, 255), 2);
        Imgproc.rectangle(input, roi2, new Scalar(0, 255, 0), 2);
        Imgproc.rectangle(input, roi3, new Scalar(255, 0, 0), 2);



        double totalArea1 = roi1.width * roi1.height;
        double totalArea2 = roi2.width * roi2.height;
        double totalArea3 = roi3.width * roi3.height;

        zoneColourPercentage[0] = (int) (100.0 * Core.countNonZero(mask.submat(roi1)) / totalArea1);
        zoneColourPercentage[1] = (int) (100.0 * Core.countNonZero(mask.submat(roi2)) / totalArea2);
        zoneColourPercentage[2] = (int) (100.0 * Core.countNonZero(mask.submat(roi3)) / totalArea3);

        } finally {
            // Release resources in finally block to ensure they're always released
            if (hsvImage != null) {
                hsvImage.release();
            }
            if (mask != null) {
                mask.release();
            }
        }

        return input;
    }
    public static void setColour(String colour) {
        allianceColour = colour;
    }
    public static int getPosition(){
        int position;
        if (zoneColourPercentage[0] > zoneColourPercentage[1] && zoneColourPercentage[0] > zoneColourPercentage[2]) {
            position = 1;
        } else if (zoneColourPercentage[1] > zoneColourPercentage[2]) {
            position = 2;
        } else {
            position = 3;
        }
        return position;
    }

    public static Scalar getMinHSV() {
        Mat roiMat = hsvImage.submat(roi1);
        Core.MinMaxLocResult minMaxLocResult = Core.minMaxLoc(roiMat);
        Scalar minHSV = new Scalar(minMaxLocResult.minVal);
        return minHSV;
    }

    public static Scalar getMaxHSV() {
        Mat roiMat = hsvImage.submat(roi1);
        Core.MinMaxLocResult minMaxLocResult = Core.minMaxLoc(roiMat);
        Scalar maxHSV = new Scalar(minMaxLocResult.maxVal);
        return maxHSV;
    }


}

