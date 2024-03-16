package org.firstinspires.ftc.teamcode.operated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.hardware.GamepadEx;

import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "NormalControl", group = "human-controls")
public class NormalControl extends BaseOperated {
    private AtomicInteger targetArmAngle = targetArmAngle =
        new AtomicInteger((int) GlobalConfig.ElbowMotorConfig.initialAngle);

    @Override
    protected void postInit() {
        GamepadEx gamepadEx = lazyGamepadEx1.get();
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

    protected void runPeriodic() {
        // Lift motor
        if (gamepad1.dpad_up) {
            hardwareManager.elbowMotor.setPower(0.5);
        } else if (gamepad1.dpad_down) {
            hardwareManager.elbowMotor.setPower(-1);
        } else {
            hardwareManager.elbowMotor.setPower(0);
        }

        // Arm Extension
        if (lazyGamepadEx1.get().isPressed(GamepadEx.Button.RIGHT_TRIGGER)) {
            hardwareManager.armExtensionServo.setPower(0.5);
        } else if (gamepad1.right_bumper) {
            hardwareManager.armExtensionServo.setPower(-1);
        } else {
            hardwareManager.armExtensionServo.setPower(0);
        }

        hardwareManager.droneLauncherHook.setPosition(gamepad1.start ? 1 : 0);
    }

    protected double getElbowPower() {
        if (lazyGamepadEx1.get().isPressed(GamepadEx.Button.LEFT_TRIGGER)) {
            targetArmAngle.getAndIncrement();
        } else if (gamepad1.left_bumper) {
            targetArmAngle.getAndDecrement();
        }

        return hardwareManager.getPowerToHoldElbowAt(targetArmAngle.get());
    }
}
