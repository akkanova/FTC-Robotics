package org.firstinspires.ftc.teamcode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.all_purpose.Misc;
import org.firstinspires.ftc.teamcode.base.HumanOperated;

/**
 * GamePad Control Schema:
 * (left_trigger) -> Upper Arm Down          (right_trigger) -> Lower Arm Down
 * (left_bumper)  -> Upper Arm Up            (right_bumper)  -> Lower Arm Up
 *     (d_pad_up) -> Claw Close            (X, Y, B, A) -> none
 *     (d_pad_down) -> Claw Open           (start) -> Launch Drone
 *              (left_stick)          (right_stick)
 *                   ║                     ╚ ▶ Forward / Reverse of Entire Right Side;
 *                   ╚ ▶ Forward / Reverse of Entire Left Side;
 */
@TeleOp(name = "Tank Control", group = "TeleOp")
public class TankControl extends HumanOperated {
    @Override
    public void processUserInput() {
        useDefaultDroneLauncherControls();
        useDefaultLiftControls();
        useDefaultArmControls();

        double leftPower   = -Misc.easeWithCubic(gamepad1.left_stick_y);
        double leftStrafe  = Misc.easeWithCubic(gamepad1.left_stick_x);
        double rightPower  = -Misc.easeWithCubic(gamepad1.right_stick_y);
        double rightStrafe = Misc.easeWithCubic(gamepad1.right_stick_x);

        frontLeftWheelPower  = leftPower  + leftStrafe;
        frontRightWheelPower = rightPower - leftStrafe;
        backLeftWheelPower   = leftPower  + rightStrafe;
        backRightWheelPower  = rightPower - rightStrafe;
    }
}
