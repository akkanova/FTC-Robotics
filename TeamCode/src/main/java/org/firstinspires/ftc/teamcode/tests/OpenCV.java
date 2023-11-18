package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.all_purpose.ComputerVision;
import org.firstinspires.ftc.teamcode.all_purpose.HardwareManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.base.SelfDriving;
import java.util.List;

@Autonomous(name = "OpenCV Test", group = "Test")
public class OpenCV extends SelfDriving {
    ComputerVision computerVision;

    @Override
    public void runOpMode() {
        hardwareManager = new HardwareManager(hardwareMap);
        computerVision = new ComputerVision(hardwareMap);

        telemetry.addLine("Open the 3 Dots on the Top-Right.");
        telemetry.addLine("Then Select 'Camera Stream'");
        telemetry.update();

        // Live Preview Only works before the loop
        computerVision.initialize(ComputerVision.Processors.TEST,true);

        waitForStart();
        runAutonomous();
    }

    @Override
    protected void runAutonomous() {
        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = computerVision.aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Intentional Idle Loop
//            for (AprilTagDetection detection : currentDetections) {
//                // Update Telemetry ...
//            }

            telemetry.update();
            idle();
        }

        computerVision.destroy();
    }
}
