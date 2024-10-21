package org.firstinspires.ftc.teamcode.pipelines.depricated;

import static org.opencv.core.Core.*;
import static org.opencv.imgproc.Imgproc.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlueCubeFinder extends OpenCvPipeline {

    public Scalar LOWER_BOUND_COLOR = new Scalar(100, 119, 97.8);
    public Scalar HIGHER_BOUND_COLOR = new Scalar(140, 255, 255);

    Telemetry telemetry;

    private String pos = "UNKNOWN";

    public BlueCubeFinder(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    @Override
    public Mat processFrame(Mat input) {

        Mat hsv_image = new Mat();
        cvtColor(input, hsv_image, COLOR_RGB2HSV);


        Mat mask = new Mat();
        inRange(hsv_image, LOWER_BOUND_COLOR, HIGHER_BOUND_COLOR, mask);

        Mat maskImg = new Mat();

        bitwise_and(input, input, maskImg, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);



        if (contours.isEmpty()){
            System.out.println("No contours found");
            return input;
        }

        // find all of the contours greater than with a height greater than 70px, and less than 100 px wide, and add them to a list
        List<MatOfPoint> filteredContours = new ArrayList<>();
        for (MatOfPoint contour : contours){
            int x, y, w, h;
            x = boundingRect(contour).x;
            y = boundingRect(contour).y;
            w = boundingRect(contour).width;
            h = boundingRect(contour).height;

            if (h > 70 && w < 100){
                filteredContours.add(contour);
            }
        }



        telemetry.addData("Number of contours", filteredContours.size());
        for (MatOfPoint contour : contours){
            int x, y, w, h;
            x = boundingRect(contour).x;
            y = boundingRect(contour).y;
            w = boundingRect(contour).width;
            h = boundingRect(contour).height;

            if (h > 70){
                rectangle(input, new Point(x, y), new Point(x + w, y + h), new Scalar(0, 255, 0), 2);
                Moments M = moments(contour);
                int cX = (int) (M.m10 / M.m00);
                int cY = (int) (M.m01 / M.m00);

                circle(input, new Point(cX, cY), 7, new Scalar(0, 0, 255), -1);

                if (cX > 0 && cX < 640/3) {
                    pos = "LEFT";
                }
                else if (cX > 640/3 && cX < 640/3 * 2) {
                    pos = "CENTER";
                }
                else if (cX > 640/3 * 2 && cX < 640) {
                    pos = "RIGHT";
                }
                else {
                    pos = "UNKNOWN";
                }

                putText(input, "WIDTH: " + w, new Point(cX - 100, cY + 50), FONT_HERSHEY_SIMPLEX, 0.75, new Scalar(255, 255, 255), 2);
//                telemetry.addData("Width", w);

//                if (cX > 300 && cX < 340) {
//                    circle(input, new Point(cX, cY), 7, new Scalar(0, 255, 0), -1);
//                    telemetry.addLine("Centered");
//                }
//                else {
//                    telemetry.addData("Not Centered: ", cX);
//                }

                telemetry.addData("Position", pos);
            }

        }

        //release all Mats
        hsv_image.release();
        mask.release();
        maskImg.release();
        hierarchy.release();

        telemetry.update();
        return input;
    }

    public String getOutput() {
        return pos;
    }
}