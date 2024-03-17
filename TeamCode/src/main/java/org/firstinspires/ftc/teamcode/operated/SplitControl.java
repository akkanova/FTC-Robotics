package org.firstinspires.ftc.teamcode.operated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.common.misc.Misc;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "SplitControl", group = "human-controls")
public class SplitControl extends BaseTeleOp {
    private AtomicBoolean disableElbowPID = new AtomicBoolean(true);
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

    @Override
    protected void runPeriodic() {
        // Lift motor
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            hardwareManager.liftMotor.setPower(0.5);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            hardwareManager.liftMotor.setPower(-1);
        } else {
            hardwareManager.liftMotor.setPower(0);
        }

        // Extension
        hardwareManager.armExtensionServo.setPower(Misc.easeWithSquare(gamepad2.right_stick_y) * -0.5);
        hardwareManager.droneLauncherHook
            .setPosition(gamepad1.start || gamepad2.start ? 0 : 1);
    }

    @Override
    protected double getElbowPower() {
        if (disableElbowPID.get()) {
            targetArmAngle.set((int) hardwareManager.getCurrentArmAngle());
            return gamepad2.left_stick_y * -0.5;

        } else {
            if (gamepad2.left_stick_y < -0.5) {
                targetArmAngle.getAndIncrement();
            } else if (gamepad2.left_stick_y > 0.5) {
                targetArmAngle.getAndDecrement();
            }

            return hardwareManager.getRequiredElbowPower(targetArmAngle.get());
        }
    }
}
