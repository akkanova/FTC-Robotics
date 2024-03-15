package org.firstinspires.ftc.teamcode.tests.debug;

import org.firstinspires.ftc.teamcode.tests.BaseTest;
import org.firstinspires.ftc.teamcode.common.hardware.GamepadEx;

/** Test for {@link GamepadEx} */
public class GamepadDebug extends BaseTest {
    @Override
    public void runOpMode() {
        initializeBaseDrive();
        waitForStart();
        GamepadEx gamepadEx = new GamepadEx(gamepad1);

//        gamepadEx.onceButtonClicked(GamepadEx.Button.A, () -> {
//            telemetry.addLine("Hello World");
//            telemetry.update();
//        });
//
//        gamepadEx.onButtonClick(GamepadEx.Button.B, () -> {
//            telemetry.addLine("sss");
//            telemetry.update();
//        });

        while(opModeIsActive()) {
            hardwareManager.elbowMotor.setPower(-gamepad1.left_stick_y);
            telemetry.addData("POS",hardwareManager.elbowMotor.getCurrentPosition());
            telemetry.update();
            gamepadEx.sync();
        }
    }
}
