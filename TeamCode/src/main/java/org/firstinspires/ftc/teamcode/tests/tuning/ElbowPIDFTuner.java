package org.firstinspires.ftc.teamcode.tests.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.common.hardware.PIDFController;
import org.firstinspires.ftc.teamcode.tests.TestTools;

import java.util.concurrent.atomic.AtomicInteger;

/**
 * Used to tune the elbow motor. Allowing it to compensate against gravity
 * and hold a steady and accurate position.

 * <a href="https://www.youtube.com/watch?v=E6H6Nqe6qJo">
 *     Based on this video.
 * </a>
 * */
public class ElbowPIDFTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        // A: I'm not going to modify the HardwareManager version so that
        // we can pass in performance hooks..
        // This will just stay as an exact copy of it with a tightly coupled
        // debug telemetry.

        TestTools testTools = new TestTools(hardwareMap, telemetry);
        DcMotorEx elbowMotor = testTools.hardwareManager.elbowMotor;

        AtomicInteger atomicTargetAngle = new AtomicInteger(45);
        PIDFController controller = new PIDFController(
                GlobalConfig.ElbowMotorConfig.P,
                GlobalConfig.ElbowMotorConfig.I,
                GlobalConfig.ElbowMotorConfig.D
        );

        waitForStart();

        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        gamepadEx.onButtonClick(GamepadEx.Button.DPAD_UP, atomicTargetAngle::getAndIncrement);
        gamepadEx.onButtonClick(GamepadEx.Button.DPAD_DOWN, atomicTargetAngle::getAndDecrement);

        while (opModeIsActive()) {
            controller.setPID(
                GlobalConfig.ElbowMotorConfig.P,
                GlobalConfig.ElbowMotorConfig.I,
                GlobalConfig.ElbowMotorConfig.D
            );

            int currentArmTicks = elbowMotor.getCurrentPosition();
            double actualAngle = currentArmTicks * GlobalConfig.ElbowMotorConfig.ANGLE_PER_TICK +
                    GlobalConfig.ElbowMotorConfig.initialAngle;
            double targetAngle = atomicTargetAngle.get();
            double targetTicks = (targetAngle - GlobalConfig.ElbowMotorConfig.initialAngle) *
                    GlobalConfig.ElbowMotorConfig.TICK_PER_ANGLE;

            // To Compensate for Gravity
            double ff  = Math.cos(Math.toRadians(targetAngle)) * GlobalConfig.ElbowMotorConfig.F;
            double pid = controller.calculate(currentArmTicks, targetTicks);

            double power = pid + ff;

            elbowMotor.setPower(power);

            testTools.telemetry.addLine("Press dpad up to increase target angle");
            testTools.telemetry.addLine("Press dpad down to decrease target angle");

            testTools.telemetry.addData("Current Angle", actualAngle);
            testTools.telemetry.addData("Target Angle", targetAngle);
            testTools.telemetry.addData("Error Angle", targetAngle - actualAngle);

            testTools.telemetry.addData("Current Position", currentArmTicks);
            testTools.telemetry.addData("Target Position", targetTicks);
            testTools.telemetry.addData("Needed Power", power);

            testTools.telemetry.update();
            gamepadEx.sync();
        }
    }
}
