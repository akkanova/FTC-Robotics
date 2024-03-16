package org.firstinspires.ftc.teamcode.operated;

import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.common.misc.Misc;

import java.util.concurrent.atomic.AtomicInteger;

public class SplitControl extends BaseOperated {
    private AtomicInteger targetArmAngle = targetArmAngle =
        new AtomicInteger((int) GlobalConfig.ElbowMotorConfig.initialAngle);

    @Override
    protected void postInit() {
        GamepadEx gamepadEx = lazyGamepadEx2.get();
        // Left Claw
        gamepadEx.onButtonClick(GamepadEx.Button.A,
                () -> hardwareManager.leftClawServo.setPosition(0.5));
        gamepadEx.onButtonUnClick(GamepadEx.Button.A,
                () -> hardwareManager.leftClawServo.setPosition(0));

        // Right Claw
        gamepadEx.onButtonClick(GamepadEx.Button.B,
                () -> hardwareManager.leftClawServo.setPosition(0.5));
        gamepadEx.onButtonUnClick(GamepadEx.Button.B,
                () -> hardwareManager.leftClawServo.setPosition(0));

        // Elbow angle presets
        gamepadEx.onButtonClick(GamepadEx.Button.Y, () -> targetArmAngle.set(135));
        gamepadEx.onButtonClick(GamepadEx.Button.X,
                () -> targetArmAngle.set((int) GlobalConfig.ElbowMotorConfig.initialAngle));
    }

    @Override
    protected void runPeriodic() {
        // Lift motor
        if (gamepad1.dpad_up) {
            hardwareManager.elbowMotor.setPower(0.5);
        } else if (gamepad1.dpad_down) {
            hardwareManager.elbowMotor.setPower(-1);
        } else {
            hardwareManager.elbowMotor.setPower(0);
        }

        hardwareManager.armExtensionServo.setPower(-Misc.easeWithSquare(gamepad2.right_stick_y));
        hardwareManager.droneLauncherHook
            .setPosition(gamepad1.start || gamepad2.start ? 1 : 0);
    }

    @Override
    protected double getElbowPower() {
        return hardwareManager.getPowerToHoldElbowAt(targetArmAngle.get()) -
            Misc.easeWithSquare(gamepad1.left_stick_y);
    }
}
