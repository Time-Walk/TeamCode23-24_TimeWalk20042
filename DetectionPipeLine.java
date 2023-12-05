package org.firstinspires.ftc.teamcode;

import static org.opencv.core.Core.max;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectionPipeLine extends OpenCvPipeline {

    Telemetry telemetry;

    public enum resultPosition{
        LEFT,
        CENTER,
        RIGHT
    }

    public int POINT_1_X_1 = 10;
    public int POINT_1_Y_1 = 10;
    public int POINT_1_X_2 = 210;
    public int POINT_1_Y_2 = 470;
    public int POINT_2_X_1 = 215;
    public int POINT_2_Y_1 = 10;
    public int POINT_2_X_2 = 425;
    public int POINT_2_Y_2 = 470;
    public int POINT_3_X_1 = 430;
    public int POINT_3_Y_1 = 10;
    public int POINT_3_X_2 = 640;
    public int POINT_3_Y_2 = 470;

    public String targetColor = "BLUE";
    int targetChannelId = 2;

    public resultPosition result = resultPosition.CENTER;

    public void initFixed(Telemetry telemetry_) {
        telemetry = telemetry_;
        switch ( targetColor ) {
            case "BLUE":
                targetChannelId = 1;
                break;
            case "RED":
                targetChannelId = 2;
                break;
        }
    }

    Mat getTargetFrame(Mat input) {
        Mat frameYCrCb = new Mat();
        Mat outputFrame = new Mat();
        Imgproc.cvtColor(input, frameYCrCb, Imgproc.COLOR_BGR2YCrCb);
        Core.extractChannel(frameYCrCb, outputFrame, targetChannelId);
        return outputFrame;
    }

    int countItemInArray(byte[] frame, byte value) {
        int cnt = 0;
        for (byte i: frame) {
            if (i == value) {
                cnt++;
            }
        }
        return cnt;
    }
    @Override
    public Mat processFrame(Mat input) {

        Mat frame = getTargetFrame(input);

        Mat rg1 = frame.submat(new Rect(new Point(POINT_1_X_1, POINT_1_Y_1), new Point(POINT_1_X_2, POINT_1_Y_2)));
        Mat rg2 = frame.submat(new Rect(new Point(POINT_2_X_1, POINT_2_Y_1), new Point(POINT_2_X_2, POINT_2_Y_2)));
        Mat rg3 = frame.submat(new Rect(new Point(POINT_3_X_1, POINT_3_Y_1), new Point(POINT_3_X_2, POINT_3_Y_2)));

        //Core.inRange(frame, new Scalar(160), new Scalar(255), rg1);
        //Core.inRange(frame, new Scalar(160), new Scalar(255), rg2);
        //Core.inRange(frame, new Scalar(160), new Scalar(255), rg3);

        double avg1 = (double) Core.mean(rg1).val[0];
        double avg2 = (double) Core.mean(rg2).val[0];
        double avg3 = (double) Core.mean(rg3).val[0];

        telemetry.addData("avg1", avg1);
        telemetry.addData("avg2", avg2);
        telemetry.addData("avg3", avg3);

        double wires = (double) Math.max(avg1, avg2);
        double max = (double) Math.max(wires, avg3);

        telemetry.addData("max", max);

        if (max == avg1) {
            result = resultPosition.LEFT;
        }else if (max == avg2) {
            result = resultPosition.CENTER;
        }else{
            result = resultPosition.RIGHT;
        }

        telemetry.addData("result", result);
        telemetry.update();

        return input;
    }

    public resultPosition getResult() {
        return result;
    }

}
