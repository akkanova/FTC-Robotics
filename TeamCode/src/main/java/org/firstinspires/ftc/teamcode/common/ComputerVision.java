package org.firstinspires.ftc.teamcode.common;

import android.util.Size;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.processors.ColorDetectionProcessor;
import org.firstinspires.ftc.teamcode.common.processors.TestProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

/**
 * Class to setup webcam stream and vision processors
 * for various object detection and classification.
 */
public class ComputerVision {
    protected final CameraName camera;
    protected final Size calibratedSize;

    protected ColorDetectionProcessor colorDetectionProcessor;
    protected AprilTagProcessor aprilTagProcessor;
    protected VisionPortal visionPortal;


    /** Create an instance with the default device name `Webcam` and calibrated resolution */
    public ComputerVision(HardwareMap hardwareMap) {
        this(hardwareMap, "Webcam", new Size(640, 480)); // For Logitech C270
    }

    /** Create an instance with the specified webcam name and resolution */
    public ComputerVision(HardwareMap hardwareMap, String webcamName, Size cameraResolution) {
        this(hardwareMap.get(WebcamName.class, webcamName), cameraResolution);
    }

    /** Create an instance with the provided Camera anc resolution */
    public ComputerVision(CameraName camera, Size cameraResolution) {
        this.calibratedSize = cameraResolution;
        this.camera = camera;
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
        public static final byte TENSORFLOW_MODEL = 4;
        public static final byte COLOR_DETECTION = 8;
    }

    /**
     * Initialize without active preview - Default Option.
     * @param enabledProcessors {@link Processors}
     */
    public void initialize(int enabledProcessors) {
        initialize(enabledProcessors, false);
    }

    /**
     * Create a vision portal and start piping the given camera stream towards
     * the enabled processors.<br/><br/>
     *
     * How to use:<br/>
     * Just enable one of the processors:<br/>
     * {@code ComputerVision.initialize(Processors.TEST, false);}<br/><br/>
     *
     * Enable two processors: <br/>
     * {@code ComputerVision.initialize(Processors.TEST | Processors.APRIL_TAG, false);}
     * {@code ComputerVision.initialize(Processors.TEST + Processors.APRIL_TAG, false);}
     *
     * @param enabledProcessors {@link Processors}
     * @param enablePreview Enable only for development - Hardware Intensive
     */
    public void initialize(int enabledProcessors, boolean enablePreview) {
        VisionPortal.Builder visionBuilder = new VisionPortal.Builder()
                .setCameraResolution(calibratedSize)
                .setAutoStopLiveView(!enablePreview)
                .enableLiveView(enablePreview)
                .setCamera(camera);

        if ((enabledProcessors & Processors.TEST) == Processors.TEST)
            visionBuilder.addProcessor(new TestProcessor());

        if ((enabledProcessors & Processors.APRIL_TAG) == Processors.APRIL_TAG) {
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawCubeProjection(enablePreview)
                    .setDrawTagOutline(enablePreview)
                    .setDrawTagID(enablePreview)
                    .setDrawAxes(enablePreview)
                    .build();

            visionBuilder.addProcessor(aprilTagProcessor);
        }

        if ((enabledProcessors & Processors.COLOR_DETECTION) == Processors.COLOR_DETECTION) {
            // Filtering for the white pixel
            colorDetectionProcessor = new ColorDetectionProcessor(
                    new Scalar(  0,   0, 178), // LOWER HSV
                    new Scalar(172, 111, 255)  // UPPER HSV
            );

            visionBuilder.addProcessor(colorDetectionProcessor);
        }

        if (enablePreview)
            FTCDashboardWrapper.addPreviewProcessor(visionBuilder);

        visionPortal = visionBuilder.build();
    }

    /** Pause camera stream */
    public void pause() {
        visionPortal.stopStreaming();
    }

    /** Resume camera stream */
    public void resume() {
        visionPortal.resumeStreaming();
    }

    /**
     * This will stop all vision related processing, shut down the camera,
     * and remove any live preview.<br/><br/>
     * After calling this function, you have to create a new instance
     * of ComputerVision to be able to use any of the processors.
     * */
    public void destroy() {
        visionPortal.close();
    }


    //----------------------------------------------------------------------------------------------
    // Getters
    //----------------------------------------------------------------------------------------------

    /** @return null if ComputerVision has not been initialized */
    public @Nullable VisionPortal getVisionPortal() {
        return visionPortal;
    }

    /** @return null if not enabled, on {@code ComputerVision.initialize()} */
    public @Nullable AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    /** @return null if not enabled, on {@code ComputerVision.initialize()} */
    public @Nullable ColorDetectionProcessor getColorDetectionProcessor() {
        return colorDetectionProcessor;
    }
}
