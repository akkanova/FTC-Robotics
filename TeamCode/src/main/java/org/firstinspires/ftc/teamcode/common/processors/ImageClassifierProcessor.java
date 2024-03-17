package org.firstinspires.ftc.teamcode.common.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

/**
 * Vision processor implementation for a Tensorflow Image Classifier.
 * An image classifier can be roughly defined as something that looks at an image
 * and tells you what it is. It is provided a limited set of labels, such as "apple",
 * "bee", or "cat", then looks at a provided image and then tells you that with an
 * X.XX % of confidence that it is one of the labels.

 * <br>CenterStage 2023-2024:<br>
 * Image classifiers used by our team were trained using
 * <a href="https://teachablemachine.withgoogle.com/">
 *      Google's Teachable Machine
 * </a>
 * with a dataset containing 80+ photos for each of the six possible positions,
 * three for each colored alliance.
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
