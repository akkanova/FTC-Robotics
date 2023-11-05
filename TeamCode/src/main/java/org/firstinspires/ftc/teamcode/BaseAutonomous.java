package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class BaseAutonomous extends Root {
    private final double MOVEMENT_POWER = 0.5;
    private final double TURN_POWER  = 0.3;

    private final double WHEEL_CIRCUMFERENCE = Math.PI * 0.098; // M
    private final int COUNTS_PER_MOTOR_REVOLUTION = 900;
    private final double COUNTS_PER_METER =
            COUNTS_PER_MOTOR_REVOLUTION / WHEEL_CIRCUMFERENCE;

    private boolean isRunning = false;
    private double frontLeftWheelP = 0;
    private double frontRightWheelP = 0;
    private double backLeftWheelP = 0;
    private double backRightWheelP = 0;

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
        sendTelemetry();
    }

    protected void sleep(double ms) {
        ElapsedTime time = new ElapsedTime();
        while(time.milliseconds() < ms) {
            // Idle stuff
            sendTelemetry();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////
    // Forward / Backward Movement
    /////////////////////////////////////////////////////////////////////////////////////

    protected void move(double distanceM) {
        if (!isRunning)
            return;

        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        doToAllWheels((wheel) -> wheel.setPower(MOVEMENT_POWER));

        double totalCounts = COUNTS_PER_METER * distanceM;
        while(isRunning && getAveragePosition() <= totalCounts) {
            // Idle loop
            sendTelemetry();
        }

        doToAllWheels((wheel) -> wheel.setPower(0));
    }

    protected double getAveragePosition() {
        return (frontLeftWheel.getCurrentPosition() +
                frontRightWheel.getCurrentPosition() +
                backLeftWheel.getCurrentPosition() +
                backRightWheel.getCurrentPosition()) / 4;

    }

    /////////////////////////////////////////////////////////////////////////////////////
    // Rotation Movement
    /////////////////////////////////////////////////////////////////////////////////////

    protected void rotate(double angle) {
        if (!isRunning)
            return;

        imu.resetYaw();
        double initialAngle = getCurrentYawAngle();

        double offset = angle > 0 ? 1 : -1;
        double leftPower = TURN_POWER * offset;
        double rightPower = TURN_POWER * -offset;

        frontLeftWheel.setPower(leftPower);
        frontRightWheel.setPower(rightPower);
        backLeftWheel.setPower(leftPower);
        backRightWheel.setPower(rightPower);

        while(isRunning && hasReachedDesiredAngle(initialAngle, angle)) {
            // Idle loop
            sendTelemetry();
        }

        doToAllWheels((wheel) -> wheel.setPower(0));
    }

    protected double getCurrentYawAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    protected boolean hasReachedDesiredAngle(double initialAngle, double turnAngle) {
        double targetAngle = initialAngle - turnAngle;
        double currentAngle = getCurrentYawAngle();

        return turnAngle > 0
                ? currentAngle > targetAngle
                : currentAngle < targetAngle;
    }

    // Remove / Disable in Competition to decrease power consumption
    private void sendTelemetry() {
        appendTotalRuntime();
        appendMotorDebugTelemetry();
        appendOrientationSensor();
        telemetry.update();
    }
}
