package org.firstinspires.ftc.teamcode.common;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Dashboard is used as an alternative to telemetry, as it has graphs
 * and active position display.
 *
 * View dashboard @ https:\\192.168.43.1:8080\dash
 *
 * The dashboard cannot be used during matches pursuant to RS09 (2021-2022 Game Manual Part 1).
 * On actual matches this class will only have empty methods, so that the code that references
 * this could still compile. But during actual development this class will actually be used
 * properly.
 *
 * DURING COMPETITION :
 * class wrapper {
 *     log(string msg) {} // empty
 * }
 *
 * DURING DEVELOPMENT :
 * class wrapper {
 *     log(string msg) {
 *         Dashboard.sendThis(msg);
 *     }
 * }
 */
public class FTCDashboardWrapper {

    /**
     * Used to send a bitmap to the FTC-Dashboard ... Allowing for active robot webcam
     * footage to also be actively displayed there.<br><br>
     *
     * Graphics drawn by `onDrawFrame` of processors does not show up on
     * the dashboard preview stream.
     */
    private static class DashboardPreviewProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame;
        public DashboardPreviewProcessor() {
            lastFrame = new AtomicReference<>(
                    Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
        }


        /** Called by Vision Portal. Initialize */
        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // Set as blank screen with the same size as the camera stream
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        /** Called by Vision Portal. Extract data from provided RGBA matrix */
        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // Convert RGBA matrix to a bitmap
            Bitmap bitmap = Bitmap.createBitmap(
                    frame.width(),
                    frame.height(),
                    Bitmap.Config.RGB_565);

            Utils.matToBitmap(frame, bitmap);
            lastFrame.set(bitmap);

            return null;
        }

        /** Called by Vision Portal. Draw graphics on the preview stream */
        @Override
        public void onDrawFrame(
                Canvas canvas,
                int onscreenWidth, int onscreenHeight,
                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                Object userContext
        ) { /* Do Nothing, does nothing */ }

        /** Called by FTC Dashboard Core to get the latest */
        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer ->
                    bitmapConsumer.accept(lastFrame.get()));
        }
    }

    /**
     * Bind the FTC Dashboard preview processor, and start dashboard web stream.
     * @param builder {@link VisionPortal.Builder}
     */
    public static void addPreviewProcessor(VisionPortal.Builder builder) {
        DashboardPreviewProcessor processor = new DashboardPreviewProcessor();
        builder.addProcessor(processor);
        FtcDashboard.getInstance().startCameraStream(processor, 0);
    }
}

/*
 * Empty FTCDashboardWrapper meant to be
 * deployed on the Control Hub :

 import org.firstinspires.ftc.vision.VisionPortal;

 public class FTCDashboardWrapper {
    public static void addPreviewProcessor(VisionPortal.Builder builder) {}
 }

 */