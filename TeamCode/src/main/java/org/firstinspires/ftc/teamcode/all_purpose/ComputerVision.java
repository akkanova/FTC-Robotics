package org.firstinspires.ftc.teamcode.all_purpose;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class ComputerVision {
    public final VisionPortal visionPortal;

    public ComputerVision(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public ComputerVision(HardwareMap hardwareMap, boolean enablePreview) {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        visionPortal = new VisionPortal.Builder()
                .enableLiveView(enablePreview)
                .setCamera(webcamName)
                .build();
    }

    public void start() {
        visionPortal.resumeStreaming();
    }
}
