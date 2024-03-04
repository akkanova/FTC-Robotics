package org.firstinspires.ftc.teamcode.tests.debug;

import org.firstinspires.ftc.teamcode.common.ComputerVision;
import org.firstinspires.ftc.teamcode.tests.BaseTest;

/**
 * Tests the specified processor using the
 * <a href="http://192.168.43.1:8080/dash">
*       FTC Dashboard Webcam preview
 * </a>
 * */
public class VisionProcessorDebug extends BaseTest {
    @Override
    public void runOpMode() throws InterruptedException {
        ComputerVision<?> processor = ComputerVision.createDefaultAprilTagCV(hardwareMap, true);
        telemetry.addLine("View preview using Driver Hub or FTC-Dashboard.");
        telemetry.update();

        waitForStart();
        // Preview only works in the Init-loop
    }
}
