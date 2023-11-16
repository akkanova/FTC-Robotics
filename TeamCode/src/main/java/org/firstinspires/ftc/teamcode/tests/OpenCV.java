package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.all_purpose.ComputerVision;
import org.firstinspires.ftc.teamcode.all_purpose.HardwareManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.base.SelfDriving;

@Autonomous(name = "OpenCV Test", group = "Test")
public class OpenCV extends SelfDriving {
    ComputerVision computerVision;

    @Override
    public void runOpMode() {
        hardwareManager = new HardwareManager(hardwareMap);
        computerVision = new ComputerVision(hardwareMap);

        waitForStart();
        runAutonomous();
    }

    @Override
    protected void runAutonomous() {
        computerVision.initialize();

        telemetry.addLine("Open the 3 Dots on the Top-Right.");
        telemetry.addLine("Then Select 'Camera Stream'");
        telemetry.update();

        while (opModeIsActive()) {
            // Intentional Idle Loop
            for (AprilTagDetection detection : computerVision.aprilTagProcessor.getDetections()) {
                // Update Telemetry ...
            }

            telemetry.update();
            idle();
        }

        computerVision.destroy();
    }
}
