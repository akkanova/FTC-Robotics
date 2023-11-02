package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "OpenCV Test", group = "Test")
public class OpenCV extends LinearOpMode {
    private final boolean SHOW_LIVE_PREVIEW = true;
    private OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        // Reference to a live preview streamed to
        // the operator's driver hub
        int cameraMonitorViewId;

        // Reference to the actual webcam attached to
        // the Control hub
        WebcamName webcamHardware = hardwareMap.get(WebcamName.class, "webcam");

        if (SHOW_LIVE_PREVIEW) {
            cameraMonitorViewId = hardwareMap.appContext
                    .getResources()
                    .getIdentifier(
                            "cameraMonitorViewId",
                            "id",
                            hardwareMap.appContext
                                    .getPackageName());

            webcam = OpenCvCameraFactory.getInstance()
                    .createWebcam(webcamHardware, cameraMonitorViewId);

        } else {
            webcam = OpenCvCameraFactory.getInstance()
                    .createWebcam(webcamHardware);
        }

        waitForStart();

        // Continue from here ...
    }
}
