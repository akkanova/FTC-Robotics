package org.firstinspires.ftc.teamcode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.HumanOperated;

/**
 * The control is split between two operators.
 * One controls the arm, the other controls
 * movement.
 *
 * Driver GamePad Control Schema (LEFT PORT, gamepad1)
 *
 *    (D_PAD)                    (Button X, Y, B, A)  ═ ▶ None
 *     ║   (Left-Stick)    (Right-Stick)
 *     ║        ║               ╚ ▶ Forward, Backward, Rotate Left or Right;
 *     ▼        ╚ ▶ For Omnidirectional Strafing;
 *     None
 *
 * Arm GamePad Control Schema (RIGHT PORT, gamepad2)
 *
 *    (D_PAD)                    (Button X, Y, B, A)  ═ ▶ None
 *     ║   (Left-Stick)    (Right-Stick)
 *     ║        ║               ╚ ▶ Up & Down controls the bottom arm servo;
 *     ▼        ╚ ▶ Up & Down controls the top arm servo;
 *     None
 *
 * Left-and-Right on BOTH GamePad2 joysticks control the claw.
 * Start Button for the both of them launches the drone.
 */
@TeleOp(name="Split Control", group = "TeleOp")
public class SplitControl extends HumanOperated {
    private final double JOYSTICK_TO_CLAW_MULTIPLIER = 0.5;

    @Override
    public void loop() {
        useDefaultDroneLauncherControl();
        useDefaultMovementControls();

        topArmServoP = gamepad2.left_stick_y * JOYSTICK_TO_CLAW_MULTIPLIER;
        bottomArmServoP = gamepad2.right_stick_y * JOYSTICK_TO_CLAW_MULTIPLIER;

        // If the left joy stick does not receive any left/right input, try reading
        // from the right joystick. Allowing claw controls to be
        // received from both left and right joysticks.

        // Claw
        if (gamepad2.dpad_left) {
            clawServoP = -SERVO_DELTA;
        } else if (gamepad2.dpad_right) {
            clawServoP = SERVO_DELTA;
        } else {
            clawServoP = 0;
        }

        clawServoP *= JOYSTICK_TO_CLAW_MULTIPLIER;

        setHardwarePower();
    }
}
