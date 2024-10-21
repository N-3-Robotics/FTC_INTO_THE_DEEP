package org.firstinspires.ftc.teamcode.pipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedPipeline extends OpenCvPipeline {

    public Scalar lower = new Scalar(0, 151.0, 86);
    public Scalar upper = new Scalar(240, 255, 160);

    private Telemetry telemetry;


    private Mat ycrcbMat       = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();

    double redAmount1 = 0;
    double redAmount2 = 0;

    private final double redThreshold = 4000;
    private String pos = "UNKNOWN";

    public RedPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat frame) {

        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(ycrcbMat, lower, upper, binaryMat);

        maskedInputMat.release();
        Core.bitwise_and(frame, frame, maskedInputMat, binaryMat);

        // Define the coordinates of three rectangles
        // You need to adjust these coordinates based on your screen resolution
        Rect rect1 = new Rect(640-121, 180, 120, 250);
        Rect rect2 = new Rect(125, 175, 180, 215);

        // Draw rectangles on the output
        drawRectangle(maskedInputMat, rect1, new Scalar(255, 0, 0)); // Blue
        drawRectangle(maskedInputMat, rect2, new Scalar(0, 255, 0)); // Green

        drawRectangle(frame, rect1, new Scalar(255, 0, 0)); // Blue
        drawRectangle(frame, rect2, new Scalar(0, 255, 0)); // Green



        // Calculate the amount of red in each rectangle
        Mat r1 = maskedInputMat.submat(rect1);
        Mat r2 = maskedInputMat.submat(rect2);
        redAmount1 = calculateRedAmount(r1);
        redAmount2 = calculateRedAmount(r2);
        r1.release();
        r2.release();


        if (redAmount1 > redThreshold) {
            pos = "RIGHT";
            telemetry.addData("Position", "RIGHT");
        } else if (redAmount2 > redThreshold) {
            pos = "CENTER";
            telemetry.addData("Position", "CENTER");
        } else {
            pos = "LEFT";
            telemetry.addData("Position", "LEFT");
        }

//        telemetry.addData("Red Amount 1", redAmount1);
//        telemetry.addData("Red Amount 2", redAmount2);

        // Output the red amounts to the console (you can modify this part)
        telemetry.update();
        return maskedInputMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {


        /*
        Paint myPaint = new Paint();
        myPaint.setColor(Color.rgb(0, 0, 0));
        myPaint.setStrokeWidth(1);

        // canvas.drawRect(100, 200, 200, 300, myPaint);
        // canvas.drawRect(350, 190, 450, 300, myPaint);
        canvas.drawRect((int) (100.0/640 * 1000), (int) (230.0/480 * 1000), (int) (200.0/640 * 1000), (int) (330.0/480 * 1000), myPaint);
        canvas.drawRect((int) (350.0/640 * 1000), (int) (230.0/480 * 1000), (int) (450.0/640 * 1000), (int) (330.0/480 * 1000), myPaint);


         */

    }


    // Helper method to calculate the amount of red in a given Mat using countNonZero
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
