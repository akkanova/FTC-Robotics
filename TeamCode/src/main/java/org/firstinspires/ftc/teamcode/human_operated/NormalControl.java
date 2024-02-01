package org.firstinspires.ftc.teamcode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.HumanOperated;

/**
 * In normal control mode, the entire control of the
 * robot is handled by a single game controller.
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
@TeleOp(name = "Normal Control", group = "TeleOp")
public class NormalControl extends HumanOperated {
    ElapsedTime timeElapsed = new ElapsedTime();
    boolean armControlOverride = false;
    @Override
    protected void processUserInput() {
        useDefaultDroneLauncherControls();
        useDefaultMovementControls();
        useDefaultLiftControls();
        useSimpleArmControls();
        checkSpeedOfEachWheel((int)timeElapsed.seconds(), 2);

        telemetry.addData("SERVO POSITION", hardwareManager.clawServoLeft.getPosition());
        telemetry.update();
        //useDefaultArmControls();
    }
}