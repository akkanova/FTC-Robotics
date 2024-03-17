package org.firstinspires.ftc.teamcode.common.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/** A processor strictly for testing */
public class TestProcessor implements VisionProcessor {

    /** Called by Vision Portal. Initialize */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    /** Called by Vision Portal. Extract data from provided RGBA matrix */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the frame into grayscale
        // Only do this for testing.. Never actually mutate the values of `frame`
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2GRAY);
        return null;
    }

    /**
     * Called by Vision Portal. Draw graphics on the preview stream.
     * `userContext` comes from the object returned by `processFrame()`
     * */
    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth, int onscreenHeight,
            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
            Object userContext
    ) {}
}
