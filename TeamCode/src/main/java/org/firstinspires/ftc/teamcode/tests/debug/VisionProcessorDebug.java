package org.firstinspires.ftc.teamcode.tests.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.ComputerVision;

/**
 * Tests the specified processor using the
 * <a href="http://192.168.43.1:8080/dash">
*       FTC Dashboard Webcam preview
 * </a>
 * */
public class VisionProcessorDebug extends LinearOpMode {
    @Override
    public void runOpMode() {
        ComputerVision<?> processor = ComputerVision.createDefaultAprilTagCV(hardwareMap, true);
        telemetry.addLine("View preview using Driver Hub or FTC-Dashboard.");
        telemetry.update();

        waitForStart();
    }
}
