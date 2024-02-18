package org.firstinspires.ftc.teamcode.common;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.common.processors.ColorDetectionProcessor;
import org.firstinspires.ftc.teamcode.common.processors.TestProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Scalar;

/** A factory class for simplifying the initialization of pre-configured vision processor pipelines. */
public final class ComputerVisionCreator {

    /** A class for containing both an active vision portal and the initialized vision-processor */
    public static class ComputerVision<T extends  VisionProcessor> {
        public final VisionPortal visionPortal;
        public final T processor;

        public ComputerVision(T processor, CameraName webcam, Size cameraResolution, boolean enablePreview) {
            this.processor = processor;
            this.visionPortal = new VisionPortal.Builder()
                .setCameraResolution(cameraResolution)
                .setAutoStopLiveView(!enablePreview)
                .enableLiveView(enablePreview)
                .addProcessor(processor)
                .setCamera(webcam)
                .build();

            if (enablePreview)
                FtcDashboard
                    .getInstance()
                    .startCameraStream(this.visionPortal, 0);
        }

        // Use Team Configured Webcam (approx 11ft off from ground)
        private ComputerVision(T processor, HardwareMap hardwareMap, boolean enabledPreview) {
            this(processor,
                    hardwareMap.get(CameraName.class, "webcam"),
                    new Size(640, 480),
                    enabledPreview);
        }

        /** Pauses the vision portal stream asynchronously. Will take a few seconds to finish this. */
        public void pause() {
            visionPortal.stopStreaming();
        }

        /**
         * Resumes the vision portal stream asynchronously if it were paused.
         * Will take a few seconds to finish this.
         * */
        public void resume() {
            visionPortal.resumeStreaming();
        }

        /**
         * Only call if no longer using this instance of ComputerVision. Destroys and releases
         * resources consumed by the {@link VisionPortal} and the selected {@link VisionProcessor}.
         * */
        public void destroy() {
            visionPortal.close();
        }
    }

    //------------------------------------------------------------------------------------------------
    // Factory Methods
    //------------------------------------------------------------------------------------------------

    /**
     * @param hardwareMap {@link HardwareMap} TeleOp provided hardware bindings
     * @return {@link ComputerVisionCreator} and {@link TestProcessor} instances, using
     * the default camera from the provided HardwareMap. The processor returned
     * is mainly used for debugging purposes. Preview is automatically enabled
     * for this processor.
     */
    public static ComputerVision<TestProcessor> createDefaultDebugCV(HardwareMap hardwareMap) {
        return new ComputerVision<>(new TestProcessor(), hardwareMap, true);
    }

    /**
     * @param hardwareMap {@link HardwareMap} TeleOp provided hardware bindings
     * @param enablePreview Only enable for debugging purposes.
     * @return {@link ComputerVisionCreator} and {@link AprilTagProcessor} instances,
     * using the default camera from the provided HardwareMap. The processor
     * returned detects april tags, and their orientation and distance
     * relative to the camera through `processor.getDetections()`.
     */
    public static ComputerVision<AprilTagProcessor> createDefaultAprilTagCV(
        HardwareMap hardwareMap,
        boolean enablePreview
    ) {
        return new ComputerVision<>(
            new AprilTagProcessor.Builder()
                .setDrawCubeProjection(enablePreview)
                .setDrawTagOutline(enablePreview)
                .setDrawTagID(enablePreview)
                .setDrawAxes(enablePreview)
                .build(),
            hardwareMap,
            enablePreview
        );
    }

    /**
     * @param hardwareMap {@link HardwareMap} TeleOp provided hardware bindings
     * @param enablePreview Only enable for debugging purposes.
     * @return {@link ComputerVisionCreator} and {@link ColorDetectionProcessor} instances,
     * using the default camera from the provided HardwareMap. The processor returned
     * has been configured to filter and detect for white pixels.
     *
     * To find the lower and upper HSV bound for this processor, use this:
     * <a href="https://github.com/PerfecXX/Python-HSV-Finder">HSV Finder</a>
     */
    public static ComputerVision<ColorDetectionProcessor> createDefaultColorDetectionCV(
        HardwareMap hardwareMap,
        boolean enablePreview
    ) {
        return new ComputerVision<>(
            new ColorDetectionProcessor(
                new Scalar(  0,   0, 178), // LOWER HSV
                new Scalar(172, 111, 255)  // UPPER HSV
            ),
            hardwareMap,
            enablePreview
        );
    }

    /**
     * @param hardwareMap {@link HardwareMap} TeleOp provided hardware bindings
     * @param enablePreview Only enable for debugging purposes.
     * @return {@link ComputerVisionCreator} and {@link TfodProcessor} instances,
     * using the default camera from the provided HardwareMap. The processor
     * returned has been trained to <b>detect</b> white pixels. Returning
     * their approximate location in relative to the camera feed.
     */
    public static ComputerVision<TfodProcessor> createDefaultTfodCV(
        HardwareMap hardwareMap,
        boolean enablePreview
    ) {
        return new ComputerVision<>(
            TfodProcessor.easyCreateWithDefaults(),
            hardwareMap,
            enablePreview
        );
    }
}