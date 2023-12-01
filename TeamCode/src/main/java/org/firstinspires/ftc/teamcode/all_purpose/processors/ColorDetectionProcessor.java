package org.firstinspires.ftc.teamcode.all_purpose.processors;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

/**
 * Currently Work In Progress ....
 */
public class ColorDetectionProcessor implements VisionProcessor {
    // Lenient bounds for filtering white objects
    private static final Scalar LOWER_HSV = new Scalar(0, 0, 178);
    private static final Scalar UPPER_HSV = new Scalar(172,111,255);
    private static final int CANVAS_PADDING_PX = 12;

    private final AtomicReference<Point> approxLocation = new AtomicReference<>();
    private final Object drawSync = new Object(); // Multi-threading ..
    private int cameraHeight;
    private int cameraWidth;

    public enum FoundRegion {
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.approxLocation.set(new Point(-1, -1));
        this.cameraHeight = height;
        this.cameraWidth = width;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert RGBA frame to HSV
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Filter according to the lower HSV and higher HSV
        Mat masked = new Mat();
        Core.inRange(hsvFrame, LOWER_HSV, UPPER_HSV, masked);

        // Additional image cleanup
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(masked, masked, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(masked, masked, Imgproc.MORPH_CLOSE, kernel);

        // Find contours of the detected white pixels
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat(); // Not really used
        Imgproc.findContours(masked, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        MatOfPoint largestContour = null;
        double maxArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        // For testing purposes only
        // Core.copyTo(masked, frame, masked);

        // Cleanup
        hsvFrame.release();
        masked.release();
        contours.clear();
        hierarchy.release();

        // Approx
        if (largestContour != null) {
            // Get its center
            Moments moments = Imgproc.moments(largestContour);
            approxLocation.get().x = moments.get_m10() / moments.get_m00();
            approxLocation.get().y = moments.get_m01() / moments.get_m00();
            largestContour.release();

            return true;
        }

        return false;
    }

    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth, int onscreenHeight,
            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
            Object userContext
    ) {
        synchronized (drawSync) {
            if (!((boolean) userContext))
                return;

            Paint paint = new Paint();
            paint.setColor(Color.RED);
            paint.setStyle(Paint.Style.FILL);
            paint.setTextSize(64);

            Point location = approxLocation.get();
            float onScreenXPos = (float) (location.x - 1.0) / cameraWidth * onscreenWidth + 1;
            float onScreenYPos = (float) (location.y - 1.0) / cameraHeight * onscreenHeight + 1;

            String label = "(" + location.x + ", " + location.y + ")";
            canvas.drawText(label, onScreenXPos, onScreenYPos, paint);
            canvas.drawCircle(onScreenXPos, onScreenYPos, 20, paint);

            paint.setStrokeWidth(12);
            paint.setStyle(Paint.Style.STROKE);

            // Assume that region returned from this is not NONE
            FoundRegion region = getRegion();

            int width = onscreenWidth / 3;
            int left = (width * region.ordinal() + CANVAS_PADDING_PX);

            canvas.drawRect(
                left, CANVAS_PADDING_PX,
                left + width - CANVAS_PADDING_PX * 2,
                onscreenHeight - CANVAS_PADDING_PX,
                paint);
        }
    }

    public FoundRegion getRegion() {
        Point location = approxLocation.get();

        if (location == null ||
            location.x < 0   ||
            location.y < 0)
            return FoundRegion.NONE;

        double width = cameraWidth / 3.0;

        if (location.x >= 0 && location.x <= width)
            return FoundRegion.LEFT;
        else if (location.x > width && location.x < width * 2)
            return FoundRegion.CENTER;
        else
            return FoundRegion.RIGHT;
    }
}

