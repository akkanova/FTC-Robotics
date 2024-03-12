package org.firstinspires.ftc.teamcode.tests.debug;

import org.firstinspires.ftc.teamcode.tests.BaseTest;

/** Will it drive straight?.. test */
public class DriveStraightDebug extends BaseTest {

    @Override
    public void runOpMode() {
        initializeBaseDrive();
        waitForStart();

        while (opModeIsActive()) {
            hardwareManager.doForAllWheels((wheel) -> {
               if (gamepad1.dpad_up) {
                   wheel.setPower(0.75);
               } else if (gamepad1.dpad_down) {
                   wheel.setPower(-0.75);
               } else {
                   wheel.setPower(0);
               }
            });
        }
    }
}
