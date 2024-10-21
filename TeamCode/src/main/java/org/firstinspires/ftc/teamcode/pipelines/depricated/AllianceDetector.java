package org.firstinspires.ftc.teamcode.pipelines.depricated;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class AllianceDetector extends OpenCvPipeline {
    private Telemetry telemetry;
    public AllianceDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /* BLUE CRAP */
    public Scalar lowerB = new Scalar(0, 0, 0);
    public Scalar upperB = new Scalar(255, 117.6, 255);

    private Mat ycrcbMatB       = new Mat();
    private Mat binaryMatB      = new Mat();
    private Mat maskedInputMatB = new Mat();

    double blueAmount = 0;
    private final double blueThreshold = 1500;

    /* END BLUE CRAP */


    /* RED CRAP */
    public Scalar lowerR = new Scalar(0, 151.0, 86);
    public Scalar upperR = new Scalar(240, 255, 160);

    private Mat ycrcbMatR       = new Mat();
    private Mat binaryMatR      = new Mat();
    private Mat maskedInputMatR = new Mat();

    double redAmount = 0;
    private final double redThreshold = 1500;

    /* END RED CRAP */


    public String alliance = "UNKNOWN";

    @Override
    public Mat processFrame(Mat input) {


        /*BLUE*/
        Imgproc.cvtColor(input, ycrcbMatB, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcbMatB, lowerB, upperB, binaryMatB);

        maskedInputMatB.release();
        Core.bitwise_and(input, input, maskedInputMatB, binaryMatB);

        // Define the coordinates of three rectangles
        // You need to adjust these coordinates based on your screen resolution
        Rect rect1 = new Rect(640-146, 180+130, 145, 115);
        Mat r1 = maskedInputMatB.submat(rect1);
        blueAmount = calculateColourAmount(r1);
        r1.release();
        /*END BLUE*/

        /*RED*/
        Imgproc.cvtColor(input, ycrcbMatR, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(ycrcbMatR, lowerR, upperR, binaryMatR);

        maskedInputMatR.release();
        Core.bitwise_and(input, input, maskedInputMatR, binaryMatR);
        r1 = maskedInputMatR.submat(rect1);
        redAmount = calculateColourAmount(r1);
        /*END RED*/

        if (redAmount > redThreshold) {
            telemetry.addData("Alliance", "RED");
            this.alliance = "RED";
        } else if (blueAmount > blueThreshold) {
            telemetry.addData("Alliance", "BLUE");
            this.alliance = "BLUE";
        }
        else {
            telemetry.addData("Alliance", "UNKNOWN");
        }

        // Draw a red rectangle if we are on the red alliance, draw blue if we are on the blue alliance
        if (alliance.equals("RED")) {
            drawRectangle(input, rect1, new Scalar(255, 0, 0));
        } else if (alliance.equals("BLUE")) {
            drawRectangle(input, rect1, new Scalar(0, 0, 255));
        }


        telemetry.addData("Red Amount", redAmount);
        telemetry.addData("Blue Amount", blueAmount);
        telemetry.update();

        return input;

    }

    private void drawRectangle(Mat mat, Rect rect, Scalar color) {
        Imgproc.rectangle(mat, rect.tl(), rect.br(), color, 2);
    }
    private double calculateColourAmount(Mat mat) {
        Mat binary = new Mat();
        Imgproc.cvtColor(mat, binary, Imgproc.COLOR_RGB2GRAY);
        Imgproc.threshold(binary, binary, 1, 255, Imgproc.THRESH_BINARY);

        int nonZeroCount = Core.countNonZero(binary);
        binary.release();

        return nonZeroCount;
    }

    public String getAlliance() {
        return this.alliance;
    }
}
