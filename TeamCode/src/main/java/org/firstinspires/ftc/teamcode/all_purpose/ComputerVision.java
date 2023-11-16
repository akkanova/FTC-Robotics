package org.firstinspires.ftc.teamcode.all_purpose;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

    // Previews are CPU intensive, so they are disabled by default..
    public void initialize() { initialize(false); }
    public void initialize(boolean enablePreview) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(enablePreview)
                .setDrawTagOutline(enablePreview)
                .setDrawTagID(enablePreview)
                .setDrawAxes(enablePreview)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCameraResolution(new Size(640, 480)) // Pre-Calibrated Resolution
                .setAutoStopLiveView(!enablePreview)
                .addProcessor(aprilTagProcessor)
                .enableLiveView(enablePreview)
                .setCamera(webcamName)
                .build();
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
}
