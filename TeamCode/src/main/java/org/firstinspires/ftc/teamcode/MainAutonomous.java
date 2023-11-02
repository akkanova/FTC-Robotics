package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Main Autonomous", group = "Autonomous")
public class MainAutonomous extends Root {
    private final double MOVEMENT_POWER = 0.5;
    private final double WHEEL_CIRCUMFERENCE = Math.PI * 0.098; // M
    private final int COUNTS_PER_MOTOR_REVOLUTION = 1440;
    private final double COUNTS_PER_METER =
            COUNTS_PER_MOTOR_REVOLUTION / WHEEL_CIRCUMFERENCE;

    private boolean isRunning = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        setupWheelMotors();
        resetWheelEncoders();
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

        move(1, 1, 5);
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
        telemetry.update();
    }

    private void move(
        double leftMeter,
        double rightMeter,
        double maxSeconds
    ) {
        if (!isRunning)
            return;

        resetWheelEncoders();

        // Make sure it doesn't do it for more than n amount of seconds
        ElapsedTime timeout = new ElapsedTime();

        // Since strafing does not work due to weight distribution issues,
        // using only tank controls would be perfectly fine.
        int leftTarget = (int) (leftMeter * COUNTS_PER_METER);
        int rightTarget = (int) (rightMeter * COUNTS_PER_METER);

        frontLeftWheel.setTargetPosition(leftTarget);
        frontRightWheel.setTargetPosition(rightTarget);
        backLeftWheel.setTargetPosition(leftTarget);
        backRightWheel.setTargetPosition(rightTarget);

        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION));
        doToAllWheels((wheel) -> wheel.setPower(MOVEMENT_POWER));

        while (isRunning && timeout.seconds() < maxSeconds &&
            (frontLeftWheel.isBusy() || frontRightWheel.isBusy() ||
             backLeftWheel.isBusy()  || backRightWheel.isBusy())) {
            // Idle loop
        }

        doToAllWheels((motor) -> motor.setPower(0));
        doToAllWheels((motor) -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }
}
