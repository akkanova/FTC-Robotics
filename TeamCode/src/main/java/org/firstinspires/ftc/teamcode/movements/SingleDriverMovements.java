package org.firstinspires.ftc.teamcode.movements;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.BaseMovements;

@TeleOp(name = "Single Driver Movements", group = "TeleOp")
/**
 * All controls of the robot can be controlled in a single
 * GamePad controller. But it is important to understand
 * within a competition environment the operator can easily
 * be overwhelmed.
 *
 * =================================IMPORTANT===================================
 * Please ensure that the GamePad controller is plugged into the furthest port
 * from the power switch. Diagram shown here:
 *
 *                 -------------------
 *                 |  Blank          |
 *                 |       Screen    |
 *                 -------------------
 *  Ethernet Port   ⮥  ⭡            ⮤ Power button Here
 *                     |
 *              ->   PLUG HERE!!  <-
 *
 * =================================IMPORTANT===================================
 *
 * GamePad Control Schema:
 * (left_trigger) -> Upper Arm Down          (right_trigger) -> Lower Arm Down
 * (left_bumper)  -> Upper Arm Up            (right_bumper)  -> Lower Arm Up
 *     (d_pad_up) -> Claw Close            (X, Y, B, A) -> none
 *     (d_pad_down) -> Claw Open           (start) -> Launch Drone
 *              (left_stick)          (right_stick)
 *                   ║                     ╚ ▶ Forward, Backward, Rotate Left or Right;
 *                   ╚ ▶ For Omnidirectional Strafing;
 */
public class SingleDriverMovements extends BaseMovements {
    // How much pressed the GamePad trigger has to be considered truthy.
    private final double GP_TRIGGER_THRESHOLD = 0.5;

    @Override
    public void loop() {
        // Upper Arm
        if (gamepad1.left_bumper) {
            topServoPower = SERVO_DELTA;
        } else if (gamepad1.left_trigger > GP_TRIGGER_THRESHOLD) {
            topServoPower = -SERVO_DELTA;
        } else {
            topServoPower = 0;
        }

        // Lower Arm
        // Side note: too heavy to maintain balance
        if (gamepad1.right_bumper) {
            bottomServoPower = SERVO_DELTA;
        } else if (gamepad1.right_trigger > GP_TRIGGER_THRESHOLD) {
            bottomServoPower = -SERVO_DELTA;
        } else {
            bottomServoPower = 0;
        }

        // Claw
        if (gamepad1.dpad_up) {
            clawPower = -SERVO_DELTA;
        } else if (gamepad1.dpad_down) {
            clawPower = SERVO_DELTA;
        } else {
            clawPower = 0;
        }

        super.loop();
    }
}
