package org.firstinspires.ftc.teamcode.all_purpose.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

/**
 * Currently Work In Progress ....
 */
public class ColorDetectionProcessor implements VisionProcessor {
    private static final Object drawSync = new Object();
    private static final int CANVAS_PADDING_PX = 0;

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
            Object userContext
    ) {
        if (userContext == null || (int)userContext == FoundRegions.NONE)
            return;

        int regions = (int) userContext;
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setAntiAlias(true);
        paint.setStyle(Paint.Style.FILL);

        if ((regions & FoundRegions.LEFT) == FoundRegions.LEFT) {

        }

        if ((regions & FoundRegions.CENTER) == FoundRegions.CENTER) {

        }

        if ((regions & FoundRegions.RIGHT) == FoundRegions.RIGHT) {

        }
    }
}
