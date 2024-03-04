package org.firstinspires.ftc.teamcode.tests.tuning;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.RawEncoder;

import org.firstinspires.ftc.teamcode.tests.BaseTest;

import java.util.List;

/**
 * Similar to ForwardPushTest, but determines the
 * ticks without having to manually push it
 * */
public class ForwardDriveTest extends BaseTest {
    private final DriveViewFactory factory;
    public ForwardDriveTest(DriveViewFactory factory) {
        this.factory = factory;
    }

    @Override
    public void runOpMode() {
        initializeDashboardTelemetry();
        DriveView driveView = this.factory.make(hardwareMap);
        waitForStart();

        double initialAveragePosition = getAverageEncoderPosition(driveView.getForwardEncs());
        while (opModeIsActive()) {
            driveView.setDrivePowers(
                new PoseVelocity2d(new Vector2d(gamepad1.left_stick_y * -0.25, 0), 0));

            double ticks = getAverageEncoderPosition(driveView.getForwardEncs()) -
                initialAveragePosition;

            telemetry.addData("ticks traveled", ticks);
            telemetry.update();
        }
    }

    private double getAverageEncoderPosition(List<RawEncoder> encoders) {
        double total = 0;
        for (RawEncoder encoder : encoders) {
            total += encoder.getPositionAndVelocity().position;
        }

        return total / encoders.size();
    }
}
