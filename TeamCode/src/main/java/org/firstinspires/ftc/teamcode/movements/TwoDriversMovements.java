package org.firstinspires.ftc.teamcode.movements;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.BaseMovements;

@TeleOp(name = "Two Drivers Movements", group = "TeleOp")
/**
 * The competition allows for two operators to operate the robot
 * simultaneously. The first operator controls basic movements
 * strafing, rotation, forward and backwards. While the second
 * operator controls the arm.
 *
 * =================================IMPORTANT===================================
 * Please ensure that both GamePad controller are plugged into the furthest port
 * from the power switch. Diagram shown here:
 *
 *                 -------------------
 *                 |  Blank          |
 *                 |       Screen    |
 *                 -------------------
 *  Ethernet Port   ⮥  ⭡  ⭡          ⮤ Power button Here
 *                     |  |
 *              ->   (Left) Drive Operator <-
 *              ->   (Right) Arm Operator <-
 *
 * =================================IMPORTANT===================================
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
 * Left-and-Right on BOTH joysticks control the claw.
 * Start Button for the both of them launches the drone.
 */
public class TwoDriversMovements extends BaseMovements {
    private final double JOYSTICK_TO_CLAW_MULTIPLIER = 0.5;

    @Override
    public void loop() {
        topServoPower = gamepad2.left_stick_y * JOYSTICK_TO_CLAW_MULTIPLIER;
        bottomServoPower = gamepad2.right_stick_y * JOYSTICK_TO_CLAW_MULTIPLIER;

        // If the left joy stick does not receive any left/right input, try reading
        // from the right joystick. Allowing claw controls to be
        // received from both left and right joysticks.
        clawPower = gamepad2.left_stick_x != 0
                ? gamepad2.left_stick_x
                : gamepad2.right_stick_x;

        clawPower *= JOYSTICK_TO_CLAW_MULTIPLIER;

        super.loop();
    }
}
