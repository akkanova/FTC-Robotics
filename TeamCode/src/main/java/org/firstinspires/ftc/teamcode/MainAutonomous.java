package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Main Autonomous", group = "Autonomous")
public class MainAutonomous extends Root {
    private final double MOVEMENT_POWER = 0.5;
    private final double WHEEL_CIRCUMFERENCE = Math.PI * 0.098; // M
    private final int COUNTS_PER_MOTOR_REVOLUTION = 900;
    private final double COUNTS_PER_METER =
            COUNTS_PER_MOTOR_REVOLUTION / WHEEL_CIRCUMFERENCE;

    private boolean isRunning = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        setupWheelMotors();
        setup9AxisSensor();
        sendInitialTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        isRunning = true;
        // Autonomous code here..
        // Since it's executed sequentially, write it here.

    }

    public void stop() {
        isRunning = false;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        appendTotalRuntime();
        appendMotorDebugTelemetry();
        appendOrientationSensor();
        telemetry.update();
    }

    private void move(double distanceM) {
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        doToAllWheels((wheel) -> wheel.setPower(MOVEMENT_POWER));

        double totalCounts = COUNTS_PER_METER * distanceM;
        while(isRunning && getAveragePosition() <= totalCounts) {
            // Idle loop
        }

        doToAllWheels((wheel) -> wheel.setPower(0));
    }

    private double getAveragePosition() {
        return (frontLeftWheel.getCurrentPosition() +
                frontRightWheel.getCurrentPosition() +
                backLeftWheel.getCurrentPosition() +
                backRightWheel.getCurrentPosition()) / 4;

    }

    private void rotate(double angle) {
        double targetAngle = getCurrentYawAngle() + angle;

        double offset = angle > 0 ? 1 : -1;
        double leftPower = MOVEMENT_POWER * offset;
        double rightPower = MOVEMENT_POWER * offset;

        frontLeftWheel.setPower(leftPower);
        frontRightWheel.setPower(rightPower);
        backLeftWheel.setPower(leftPower);
        backRightWheel.setPower(rightPower);

        while(isRunning && Math.abs(getCurrentYawAngle()) > Math.abs(targetAngle)) {
            // Idle loop
        }

        doToAllWheels((wheel) -> wheel.setPower(0));
    }

    private double getCurrentYawAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
