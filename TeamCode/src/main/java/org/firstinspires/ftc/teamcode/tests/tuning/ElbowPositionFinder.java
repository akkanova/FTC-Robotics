package org.firstinspires.ftc.teamcode.tests.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.tests.TestTools;

/**
 * Before starting the test, hold the arm at a near 90 degree angle, then start it and let go.
 * Using the angle traveled, we can determine the it's lowest position's angle (a.k.a normal
 * starting position).
 * */
public class ElbowPositionFinder extends LinearOpMode {
    @Override
    public void runOpMode() {
        TestTools tools = new TestTools(hardwareMap, telemetry);
        DcMotorEx elbowMotor = tools.hardwareManager.elbowMotor;

        tools.telemetry.addLine("Please hold the arm at a near 90 degree angle.");
        tools.telemetry.update();

        waitForStart();

        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Don't know how long it will fall for.
        while (opModeIsActive()) {
            int currentArmTicks = elbowMotor.getCurrentPosition();
            double actualAngle = currentArmTicks * GlobalConfig.ElbowMotorConfig.ANGLE_PER_TICK;

            telemetry.addLine("Let go of the arm, and wait for it to reach it's lowest position.");
            telemetry.addData("Calculated Initial Angle", 90 - actualAngle);
            telemetry.update();
        }
    }
}
