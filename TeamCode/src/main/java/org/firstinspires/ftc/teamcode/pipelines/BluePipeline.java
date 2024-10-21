package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class BluePipeline extends OpenCvPipeline {

    public BluePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Scalar lower = new Scalar(0, 0, 0);
    public Scalar upper = new Scalar(255, 117.6, 255);

    private Telemetry telemetry;


    private Mat ycrcbMat       = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();

    double redAmount1 = 0;
    double redAmount2 = 0;
    private final double redThreshold = 8000;
    private String pos = "UNKNOWN";

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(ycrcbMat, lower, upper, binaryMat);

        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        // Define the coordinates of three rectangles
        // You need to adjust these coordinates based on your screen resolution
        Rect rect1 = new Rect(640-121, 180, 120, 250);
        Rect rect2 = new Rect(125, 175, 180, 215);

        // Draw rectangles on the output frame
        drawRectangle(maskedInputMat, rect1, new Scalar(255, 0, 0)); // Blue
        drawRectangle(maskedInputMat, rect2, new Scalar(0, 255, 0)); // Green


        Mat r1 = maskedInputMat.submat(rect1);
        Mat r2 = maskedInputMat.submat(rect2);
        // Calculate the amount of red in each rectangle
        redAmount1 = calculateRedAmount(r1);
        redAmount2 = calculateRedAmount(r2);
        r1.release();
        r2.release();


        if (redAmount1 > redThreshold && redAmount1 > redAmount2) {
            pos = "RIGHT";
            telemetry.addData("Position", "RIGHT");
        } else if (redAmount2 > redThreshold && redAmount2 > redAmount1) {
            pos = "CENTER";
            telemetry.addData("Position", "CENTER");
        } else {
            pos = "LEFT";
            telemetry.addData("Position", "LEFT");
        }

        telemetry.addData("Blue Amount R", redAmount1);
        telemetry.addData("Blue Amount C", redAmount2);

        // Output the red amounts to the console (you can modify this part)
        telemetry.update();
        return maskedInputMat;
    }

    private double calculateRedAmount(Mat mat) {
        Mat binary = new Mat();
        Imgproc.cvtColor(mat, binary, Imgproc.COLOR_RGB2GRAY);
        Imgproc.threshold(binary, binary, 1, 255, Imgproc.THRESH_BINARY);

        int nonZeroCount = Core.countNonZero(binary);
        binary.release();

        return nonZeroCount;
    }
    public double getRedAmount1() {
        return redAmount1;
    }

    public double getRedAmount2() {
        return redAmount2;
    }
    private void drawRectangle(Mat mat, Rect rect, Scalar color) {
        Imgproc.rectangle(mat, rect.tl(), rect.br(), color, 2);
    }

    public String getOutput() {
        return pos;
    }

}
