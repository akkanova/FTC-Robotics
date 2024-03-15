package org.firstinspires.ftc.teamcode.tests.debug;

import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.tests.BaseTest;

/**
 * Set all wheel motors to the same power, and see
 * physically if it curves or drives straight.
 * */
public class DriveStraightDebug extends BaseTest {

    @Override
    public void runOpMode() {
        HardwareManager manager = getHardwareManager();
        MecanumDrive drive = getMecanumDrive();

        initializeDashboardTelemetry();
        waitForStart();

        while (opModeIsActive()) {
            manager.doForAllWheels((wheel) -> {
               if (gamepad1.dpad_up) {
                   wheel.setPower(0.75);
               } else if (gamepad1.dpad_down) {
                   wheel.setPower(-0.75);
               } else {
                   wheel.setPower(0);
               }
            });

            telemetry.addData("heading (deg)", Math.toDegrees(drive.currentPose.heading.toDouble()));
            telemetry.update();

            sendLocationTelemetryPacket();
        }
    }
}
