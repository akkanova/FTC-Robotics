package org.firstinspires.ftc.teamcode.operated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.hardware.GamepadEx;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "NormalControl", group = "human-controls")
public class NormalControl extends BaseTeleOp {
    private AtomicBoolean disableElbowPID = new AtomicBoolean();
    private AtomicInteger targetArmAngle =
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
            () -> hardwareManager.rightClawServo.setPosition(0.5));
        gamepadEx.onButtonUnClick(GamepadEx.Button.B,
            () -> hardwareManager.rightClawServo.setPosition(0));

        // Elbow angle presets
        gamepadEx.onButtonClick(GamepadEx.Button.Y, () -> targetArmAngle.set(135));
        gamepadEx.onButtonClick(GamepadEx.Button.X,
            () -> targetArmAngle.set((int) GlobalConfig.ElbowMotorConfig.initialAngle));

        // Disable Elbow PID
        gamepadEx.onButtonClick(GamepadEx.Button.BACK, () -> {
            hardwareManager.elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            disableElbowPID.set(!disableElbowPID.get());
        });
    }

    protected void runPeriodic() {
        // Lift motor
        if (gamepad1.dpad_up) {
            hardwareManager.liftMotor.setPower(0.5);
        } else if (gamepad1.dpad_down) {
            hardwareManager.liftMotor.setPower(-1);
        } else {
            hardwareManager.liftMotor.setPower(0);
        }

        // Arm Extension
        if (lazyGamepadEx1.get().isPressed(GamepadEx.Button.RIGHT_TRIGGER)) {
            hardwareManager.armExtensionServo.setPower(0.5);
        } else if (gamepad1.right_bumper) {
            hardwareManager.armExtensionServo.setPower(-0.5);
        } else {
            hardwareManager.armExtensionServo.setPower(0);
        }

        // Arm Wrist
        if (gamepad1.dpad_left) {
            hardwareManager.armWristServo.setPower(0.5);
        } else if (gamepad1.dpad_right) {
            hardwareManager.armWristServo.setPower(-0.5);
        } else {
            hardwareManager.armWristServo.setPower(0);
        }

        hardwareManager.droneLauncherHook.setPosition(gamepad1.start ? 0 : 1);
    }

    protected double getElbowPower() {
        if (disableElbowPID.get()) {
            targetArmAngle.set((int) hardwareManager.getCurrentArmAngle());
            return gamepad1.left_bumper
              ? 0.5 : gamepad1.left_trigger * -0.5;

        } else {
            if (lazyGamepadEx1.get().isPressed(GamepadEx.Button.LEFT_TRIGGER)) {
                targetArmAngle.getAndIncrement();
            } else if (gamepad1.left_bumper) {
                targetArmAngle.getAndDecrement();
            }
            return hardwareManager.getRequiredElbowPower(targetArmAngle.get());
        }
    }
}
