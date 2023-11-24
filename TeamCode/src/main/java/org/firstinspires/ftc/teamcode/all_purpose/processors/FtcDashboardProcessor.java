package org.firstinspires.ftc.teamcode.all_purpose.processors;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Used to send a bitmap to the FTC-Dashboard ... Allowing for active robot webcam
 * footage to also be actively displayed there.<br><br>
 *
 * View dashboard @ https:\\192.168.43.1:8080\dash
 */
public class FtcDashboardProcessor implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame;
    public FtcDashboardProcessor() {
        lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    }

    public static FtcDashboardProcessor create() {
        FtcDashboardProcessor processorInstance = new FtcDashboardProcessor();
        FtcDashboard.getInstance().startCameraStream(processorInstance, 0);
        return processorInstance;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Set as blank screen with the same size as the camera stream
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert rgb matrix to a bitmap
        Bitmap bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, bitmap);
        lastFrame.set(bitmap);

        return null;
    }

    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth, int onscreenHeight,
            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
            Object userContext
    ) {}

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
