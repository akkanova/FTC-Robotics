package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.base.HumanOperated;

/**
 * To test whether the connected GamePad has stick drift
 * in any of it's joysticks.
 *
 * Possible future feature: automatically compensate for
 * stick drift?
 */
@TeleOp(name = "GamePad Drift Test", group = "Test")
public class GamePadDrift extends HumanOperated {
    @Override
    public void loop() {
        // .. Required
    }

    @Override
    public void start() {
        telemetry.addLine("Please Do Not touch GamePads");
        telemetry.addLine("Beginning Test");
        telemetry.update();

        String result =
            gamepad1.left_stick_x != 0 ||
            gamepad1.left_stick_y != 0 ||
            gamepad1.right_stick_x != 0 ||
            gamepad1.right_stick_y != 0 ||

            gamepad2.left_stick_x != 0 ||
            gamepad2.left_stick_y != 0 ||
            gamepad2.right_stick_x != 0 ||
            gamepad2.right_stick_y != 0
                    ? "--- TEST FAILED ---"
                    : "--- TEST SUCCESS ---";

        telemetry.addLine(result);
        telemetry.update();
    }
}
