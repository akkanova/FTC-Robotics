package org.firstinspires.ftc.teamcode.all_purpose.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

/**
 * Currently Work In Progress ....
 */
public class ColorDetectionProcessor implements VisionProcessor {
    public static class FoundRegions {
        public static final byte NONE = 0;
        public static final byte LEFT = 1;
        public static final byte CENTER = 2;
        public static final byte RIGHT = 4;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        int result = FoundRegions.NONE;
        return result;
    }

    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth, int onscreenHeight,
            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
            Object result
    ) {
        if (result == null || (int)result == FoundRegions.NONE)
            return;


    }
}
