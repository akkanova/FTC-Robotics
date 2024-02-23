package org.firstinspires.ftc.teamcode.tests.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.ComputerVisionCreator;

/**
 * Tests the specified processor using the
 * <a href="http://192.168.43.1:8080/dash">
*       FTC Dashboard Webcam preview
 * </a>
 * */
public class VisionProcessorDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ComputerVisionCreator.ComputerVision<?> processor =
            ComputerVisionCreator.createDefaultAprilTagCV(hardwareMap, true);

        waitForStart();
        // Preview only works in the Init-loop
    }
}
