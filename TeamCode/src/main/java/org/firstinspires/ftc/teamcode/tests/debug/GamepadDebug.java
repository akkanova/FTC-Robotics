package org.firstinspires.ftc.teamcode.tests.debug;

import org.firstinspires.ftc.teamcode.tests.BaseTest;


/** Used to debug both user gamepads */
public class GamepadDebug extends BaseTest {
    @Override
    public void runOpMode() {
        initializeDashboardTelemetry();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("gamepad1", gamepad1);
            telemetry.addData("gamepad2", gamepad2);
            telemetry.update();
        }
    }
}
