package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.SelfDriving;

/**
 * Testing for Wrist
 */
@Autonomous(name = "Wrist Movement Test", group = "Test")
public class WristMovement extends SelfDriving {
    @Override
    protected void runAutonomous() {
        moveWristTillSeconds(1000);
    }

    private void moveWristTillSeconds(double ms) {
        if (!opModeIsActive())
            return;

        ElapsedTime elapsedTime = new ElapsedTime();
        double requiredTimeMS = ms * WRIST_GEAR_RATIO;
        hardwareManager.topLeftArmServo.setPower(MOVEMENT_POWER);
        hardwareManager.topRightArmServo.setPower(MOVEMENT_POWER);

        while(opModeIsActive() && elapsedTime.milliseconds() < requiredTimeMS) {
            idle();
        }

        hardwareManager.topLeftArmServo.setPower(0);
        hardwareManager.topRightArmServo.setPower(0);
    }
}
