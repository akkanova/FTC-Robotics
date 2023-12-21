package org.firstinspires.ftc.teamcode.common.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

/**
 * Vision processor implementation for a Tensorflow Image Classifier. Completely different from
 * the TfodProcessor, which is only a vision pipeline for a Tensorflow object detection model. <br>
 *
 * <br>CenterStage 2023-2024:<br>
 * Image classifiers used by our team were trained by google's teachable machine with a dataset
 * containing 80+ photos for each of the six possible positions, three for each colored alliance.
 */
public class ImageClassifierProcessor implements VisionProcessor {
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        //
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth,
            int onscreenHeight,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            Object userContext
    ) {
        //
    }
}
