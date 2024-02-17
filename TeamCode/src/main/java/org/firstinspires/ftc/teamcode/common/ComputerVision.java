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

// Robot Dimension
// 1ft across (not Including wheel)
// 1ft long

// Camera approx 11 ft from ground

public class ComputerVision<T extends VisionProcessor> {
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

    private static final String DEFAULT_WEBCAM_NAME = "webcam";
    private static final Size DEFAULT_WEBCAM_RESOLUTION = new Size(640, 480);

    // Use Team Configured Webcam
    private ComputerVision(T processor, HardwareMap hardwareMap, boolean enabledPreview) {
        this(processor,
                hardwareMap.get(CameraName.class, DEFAULT_WEBCAM_NAME),
                DEFAULT_WEBCAM_RESOLUTION,
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


    //------------------------------------------------------------------------------------------------
    // Factory Methods
    //------------------------------------------------------------------------------------------------

    /**
     * @param hardwareMap {@link HardwareMap} TeleOp provided hardware bindings
     * @return {@link ComputerVision} and {@link TestProcessor} instances, using
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
     * @return {@link ComputerVision} and {@link AprilTagProcessor} instances,
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
     * @return {@link ComputerVision} and {@link ColorDetectionProcessor} instances,
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
     * @return {@link ComputerVision} and {@link TfodProcessor} instances,
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

//    Game Manual Zero: https://gm0.org
//    FTCLib: https://docs.ftclib.org
//    EasyOpenCV: https://github.com/OpenFTC/EasyOpenCV
//    FTCDashboard: https://acmerobotics.github.io/ftc-da...
//    Road Runner: https://github.com/acmerobotics/road-...
//    Learn Road Runner: https://learnroadrunner.com
//    Ctrl + Alt + FTC: https://www.ctrlaltftc.com
//    Homeostasis: https://github.com/Thermal-Equilibriu...
//    Photon: https://github.com/Eeshwar-Krishnan/P...
//    BetterSensors: https://github.com/Brickwolves/Better...