package org.firstinspires.ftc.teamcode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.all_purpose.Misc;
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
    private static final double JOYSTICK_TO_CLAW_MULTIPLIER = 0.5;

    @Override
    public void processUserInput() {
        useDefaultDroneLauncherControls();
        useDefaultMovementControls();
        useDefaultLiftControls();

        topArmServoPower    = Misc.easeWithCubic(gamepad2.left_stick_y)  * JOYSTICK_TO_CLAW_MULTIPLIER;
        bottomArmMotorPower = Misc.easeWithCubic(gamepad2.right_stick_y) * JOYSTICK_TO_CLAW_MULTIPLIER;
        clawServoPower = 0;

        // Claw
        if (gamepad2.dpad_right)
            clawServoPower = ARM_SERVO_DELTA;
        else if (gamepad2.dpad_left)
            clawServoPower = -ARM_SERVO_DELTA;

        clawServoPower *= JOYSTICK_TO_CLAW_MULTIPLIER;
    }
}
