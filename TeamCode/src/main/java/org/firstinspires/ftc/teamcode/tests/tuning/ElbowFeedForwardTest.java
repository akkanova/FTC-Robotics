package org.firstinspires.ftc.teamcode.tests.tuning;

import org.firstinspires.ftc.teamcode.common.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.common.hardware.PIDFController;
import org.firstinspires.ftc.teamcode.tests.BaseTest;

import java.util.concurrent.atomic.AtomicInteger;


public class ElbowFeedForwardTest extends BaseTest {
    public static double P = 0.031;
    public static double I = 0.5;
    public static double D = 1e-5;
    public static double F = 0;

    protected final double TICK_PER_ANGLE = 1440.0 * 2 / 360.0;
    protected final double ANGLE_PER_TICK = 1 / TICK_PER_ANGLE;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeBaseDrive();
        initializeDashboardTelemetry();

        waitForStart();

        final AtomicInteger atomicTargetAngle = new AtomicInteger(45);
        final PIDFController controller = new PIDFController(P, I, D);

        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        gamepadEx.onButtonClick(GamepadEx.Button.DPAD_UP, atomicTargetAngle::getAndIncrement);
        gamepadEx.onButtonClick(GamepadEx.Button.DPAD_DOWN, atomicTargetAngle::getAndDecrement);

        while (opModeIsActive()) {
            controller.setPID(P, I, D);
            int currentArmTicks = hardwareManager.elbowMotor.getCurrentPosition();

            // Initially starts at 45 degrees off the ground.
            double actualAngle = currentArmTicks * ANGLE_PER_TICK - 45;
            double targetAngle = atomicTargetAngle.get();

            double targetTicks = targetAngle * TICK_PER_ANGLE;
            double pid = controller.calculate(currentArmTicks, targetTicks);
            double ff  = Math.cos(Math.toRadians(targetAngle)) * F;

            double power = pid + ff;

            hardwareManager.elbowMotor.setPower(power);

            telemetry.addLine("Press dpad up to increase target angle");
            telemetry.addLine("Press dpad down to decrease target angle");
            telemetry.addData("Current Angle", actualAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Error Angle", targetAngle - actualAngle);

            telemetry.addData("Current Position", currentArmTicks);
            telemetry.addData("Target Position", targetTicks);
            telemetry.addData("Needed Power", power);

            telemetry.update();
            gamepadEx.sync();
        }
    }
}
