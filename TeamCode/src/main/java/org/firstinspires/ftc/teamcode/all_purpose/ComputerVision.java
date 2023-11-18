package org.firstinspires.ftc.teamcode.all_purpose;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Contains all the processors, and setup code for Computer Vision.
 * Computer Vision Pipeline Overly-Simplified:
 *
 *   WEBCAM-SNAPSHOT-IMAGE  ->  APRIL-TAG-PROCESSOR  ->  FINAL-IMAGE-WITH-BOXES & MAGICAL-POS-DATA
 */
public class ComputerVision {
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;

    private final WebcamName webcamName;

    public ComputerVision(HardwareMap hardwareMap) {
        this.webcamName = hardwareMap.get(WebcamName.class, "webcam");
    }

    /**
     * Processors Option regarding which processors to initialize.
     * As some vision processors (e.g. {@link AprilTagProcessor}) require
     * heavy hardware resources even when not required..<br><br>
     *
     * Utilizing Bits we can enable and disable specific processors without
     * having to add more convoluted parameters. So instead of code being like this :<br>
     * {@code initialize(bool enableProcessorA, bool enableProcessorB, bool e....)} <br>
     * in can just be this:
     * {@code initialize(int mode)} <br><br>
     *
     * How does it work? - Bitwise Operators<br>
     *
     * Integer           |  Binary<br>
     * 1                 |  ... 0001<br>
     * 2                 |  ... 0010<br><br>
     *
     * Utilizing the power of Bitwise Operators, we can detect whether a specific bit in the given
     * integer is 0 or 1. Meaning each bit within an integer can become an independent boolean
     * for a corresponding option. An Integer has 32 bits (1 used for sign - or +, only 31 usable),
     * meaning 31 different options.
     */
    public static class Processors {
        public static final byte TEST = 1;
        public static final byte APRIL_TAG = 2;
        public static final byte PIXEL = 4;
    }

    /** Previews are CPU intensive, so they are disabled by default.. **/
    public void initialize(int processors) {
        initialize(processors, false);
    }

    /**
     * @param processors Refer to the documentation on {@link Processors} for more explanation.
     * @param enablePreview Should only be enabled for TESTING / DEBUGGING
     */
    public void initialize(int processors, boolean enablePreview) {
        VisionPortal.Builder visionBuilder = new VisionPortal.Builder()
            .setCameraResolution(new Size(640, 480)) // Pre-Calibrated Resolution
            .setAutoStopLiveView(!enablePreview)
            .enableLiveView(enablePreview)
            .setCamera(webcamName);

        if ((processors & Processors.TEST) == Processors.TEST) {
            visionBuilder.addProcessor(new TestProcessor());
        }

        if ((processors & Processors.APRIL_TAG) == Processors.APRIL_TAG) {
            aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(enablePreview)
                .setDrawTagOutline(enablePreview)
                .setDrawTagID(enablePreview)
                .setDrawAxes(enablePreview)
                .build();
            visionBuilder.addProcessor(aprilTagProcessor);
        }

        if ((processors & Processors.PIXEL) == Processors.PIXEL) {
            // .. Initialize a TFOD Model
        }

        visionPortal = visionBuilder.build();
    }

    public void pause() {
        visionPortal.stopStreaming();
    }

    public void resume() {
        visionPortal.resumeStreaming();
    }

    public void destroy() {
        visionPortal.close();
    }


    //------------------------------------------------------------------------------------------------
    // Processors
    //------------------------------------------------------------------------------------------------

    private class TestProcessor implements VisionProcessor {
        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
        @Override
        public void init(int width, int height, CameraCalibration calibration) {}

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // Grayscale it
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2GRAY);
            return frame;
        }
    }
}
