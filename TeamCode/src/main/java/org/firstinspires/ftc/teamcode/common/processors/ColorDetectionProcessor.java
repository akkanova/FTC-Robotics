package org.firstinspires.ftc.teamcode.common.processors;

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
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

/**
 * A processor that filters out a specific band of HSV, and returns the location of
 * the largest blob of the filtered colour in relative of the screen.
 */
public class ColorDetectionProcessor implements VisionProcessor {
    protected final AtomicReference<Point> approxLocation;
    protected final Scalar lowerHSV;
    protected final Scalar upperHSV;

    protected int cameraHeight;
    protected int cameraWidth;

    private static final int CANVAS_PADDING_PX = 12;
    private final Object drawSync; // For multi-threading sync..

    /** The arbitrary regions that the object might be in. */
    public enum FoundRegion {
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }

    /**
     * Create an instance with the given lower and upper HSV
     *
     * @param lowerHSV   Lower range of the band of color to filter for.
     * @param higherHSV  Upper range of the band of color to filter for.
     * */
    public ColorDetectionProcessor(Scalar lowerHSV, Scalar higherHSV) {
        this.approxLocation = new AtomicReference<>(new Point(-1, -1));
        this.upperHSV = higherHSV;
        this.lowerHSV = lowerHSV;
        this.drawSync = new Object();
    }

    /** Called by Vision Portal. Initialize */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.cameraHeight = height;
        this.cameraWidth = width;
    }

    /** Called by Vision Portal. Extract data from provided RGBA matrix */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert RGBA frame to HSV
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Filter according to the lower HSV and higher HSV
        Mat masked = new Mat();
        Core.inRange(hsvFrame, lowerHSV, upperHSV, masked);

        // Additional image cleanup
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(masked, masked, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(masked, masked, Imgproc.MORPH_CLOSE, kernel);

        // Find contours of the detected white pixels
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat(); // Not really used
        Imgproc.findContours(masked, contours, hierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);

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

    /** Called by Vision Portal. Draw graphics on the preview stream */
    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth, int onscreenHeight,
            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
            Object userContext
    ) {
        // Only one thread should be drawing the graphics at a time..
        // The FTC dashboard and Driver Hub could call this method simultaneously.
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

    /**
     * Get the center x and y of the largest contour and make an arbitrary
     * decision on which region of the camera feed (LEFT, CENTER, RIGHT) it is
     * likely located in.
     * @return {@link FoundRegion}
     */
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

    /**
     * @return A copy of the x and y location of the largest contour center
     * detected from the filtered camera feed.
     * */
    public Point getApproxLocation() {
        return approxLocation.get().clone();
    }
}
