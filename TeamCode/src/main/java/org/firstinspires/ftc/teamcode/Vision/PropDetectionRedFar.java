package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class PropDetectionRedFar implements VisionProcessor {
    public int detection = 2;

    public static int leftRectX1 = 470, leftRectY1 = 140;
    public static int leftRectX2 = 545, leftRectY2 = 260;

    public static double leftThresh = 800000;
    public double leftSum = 0;

    public static int middleRectX1 = 160, middleRectY1 = 170;
    public static int middleRectX2 = 240, middleRectY2 = 275;

    public static double middleThresh = 600000;
    public double middleSum = 0;

    public static int redLowH = 110, redLowS = 160, redLowV = 0;
    public static int redHighH = 125, redHighS = 255, redHighV = 255;

    Mat workingMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, workingMat, Imgproc.COLOR_BGR2HSV);

        Rect leftRect = new Rect(new Point(leftRectX1, leftRectY1), new Point(leftRectX2, leftRectY2));
        Rect middleRect = new Rect(new Point(middleRectX1, middleRectY1), new Point(middleRectX2, middleRectY2));

        Scalar lowThresh = new Scalar(redLowH, redLowS, redLowV);
        Scalar highThresh = new Scalar(redHighH, redHighS, redHighV);

        Core.inRange(workingMat, lowThresh, highThresh, workingMat);

        leftSum = Core.sumElems(workingMat.submat(leftRect)).val[0];
        middleSum = Core.sumElems(workingMat.submat(middleRect)).val[0];

        Imgproc.rectangle(frame, leftRect, new Scalar(0,255,0), 5);
        Imgproc.rectangle(frame, middleRect, new Scalar(0,255,0), 5);

        if(leftSum > leftThresh)
            detection = 1;
        else if (middleSum > middleThresh)
            detection = 2;
        else detection = 3;

//        workingMat.copyTo(frame);

        workingMat.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
