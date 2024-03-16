package org.firstinspires.ftc.teamcode.tests.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tests.TestTools;

/**
 * Set all wheel motors to the same power, and see
 * physically if it curves or drives straight.
 * */
public class DriveStraightDebug extends LinearOpMode {
    @Override
    public void runOpMode() {
        TestTools testTools = new TestTools(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive()) {
            testTools.hardwareManager.doForAllWheels((wheel) -> {
               if (gamepad1.dpad_up) {
                   wheel.setPower(0.75);
               } else if (gamepad1.dpad_down) {
                   wheel.setPower(-0.75);
               } else {
                   wheel.setPower(0);
               }
            });

            testTools.telemetry.addData("heading (deg)",
                Math.toDegrees(testTools.drive.currentPose.heading.toDouble()));

            testTools.sendPositionTelemetry();
            testTools.telemetry.update();
        }
    }
}
