package org.firstinspires.ftc.teamcode.tests.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tests.TestTools;

/** Used to debug both user gamepads */
public class GamepadDebug extends LinearOpMode {
    @Override
    public void runOpMode() {
        TestTools testTools = new TestTools(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive()) {
            testTools.telemetry.addData("gamepad1", gamepad1);
            testTools.telemetry.addData("gamepad2", gamepad2);
            testTools.telemetry.update();
        }
    }
}
