package org.firstinspires.ftc.teamcode.tests.tuning;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.tests.BaseTest;

/**
 * Before starting the test, hold the arm at a near 90 degree angle, then start it and let go.
 * Using the angle traveled, we can determine the it's lowest position's angle (a.k.a normal
 * starting position).
 * */
public class ElbowPositionFinder extends BaseTest {
    @Override
    public void runOpMode() {
        HardwareManager hardwareManager = getHardwareManager();
        telemetry.addLine("Please hold the arm at a near 90 degree angle.");
        telemetry.update();

        waitForStart();

        hardwareManager.elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardwareManager.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareManager.elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Don't know how long it will fall for.
        while (opModeIsActive()) {
            int currentArmTicks = hardwareManager.elbowMotor.getCurrentPosition();
            double actualAngle = currentArmTicks * GlobalConfig.ElbowMotorConfig.ANGLE_PER_TICK;

            telemetry.addLine("Let go of the arm, and wait for it to reach it's lowest position.");
            telemetry.addData("Calculated Initial Angle", 90 - actualAngle);
            telemetry.update();
        }
    }
}
