package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class BaseMovements extends Root {
    // Movement Config
    // (0.50 => 50% of Max Speed,
    // -0.75 => 75% of Max Speed in the opposite direction)
    protected final double MAX_REVERSE_SPEED = -0.7;
    protected final double MAX_FORWARD_SPEED = 0.7 ;
    protected final double SERVO_DELTA = 0.7;

    // temporary(?) variable for testing continuous servo use
    protected double topServoPower = 0;
    protected double bottomServoPower = 0;
    protected double clawPower = 0;
    protected double planePower = 0;

    @Override
    public void init() {
        setupWheelMotors();
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        setupArmHardware();
        setup9AxisSensor();
        sendInitialTelemetry();
    }

    @Override
    public void loop() {
        // Joystick value
        //      1 (y-value)
        //      ⭡
        // -1 ⭠ 0 ⭢ 1 (x-value)
        //      ⭣
        //     -1

        /////////////////////////////////////////////////////////////////////////////////
        // Wheel Motors Controls
        /////////////////////////////////////////////////////////////////////////////////

        // If the left joy stick does not receive any up/down input, try reading
        // from the right joystick. Allowing forward & back controls to be
        // received from both left and right joysticks.
        double drive = -((gamepad1.left_stick_y != 0)
                ? gamepad1.left_stick_y
                : gamepad1.right_stick_y);

        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Percentage speed Capped at (MAX_REVERSE_SPEED and MAX_FORWARD_SPEED)
        double frontLeftP  = limitPower(drive - strafe + rotate);
        double frontRightP = limitPower(drive - strafe - rotate);
        double backLeftP   = limitPower(drive + strafe + rotate);
        double backRightP  = limitPower(drive + strafe - rotate);

        frontLeftWheel.setPower(frontLeftP);
        frontRightWheel.setPower(frontRightP);
        backLeftWheel.setPower(backLeftP);
        backRightWheel.setPower(backRightP);

        /////////////////////////////////////////////////////////////////////////////////
        // Plane Launcher Controls
        /////////////////////////////////////////////////////////////////////////////////

        if (gamepad1.start || gamepad2.start) {
            planePower -= SERVO_DELTA;
        } else {
            planePower = 0;
        }


        topLeftArmServo.setPower(topServoPower);
        topRightArmServo.setPower(topServoPower);
        bottomLeftArmServo.setPower(bottomServoPower);
        bottomRightArmServo.setPower(bottomServoPower);

        clawServo.setPower(clawPower);
        planeServo.setPower(planePower);

        // Replace with something else in official competitions
        appendMotorDebugTelemetry();
        appendServoDebugTelemetry();
        appendOrientationSensor();
        telemetry.update();
    }

    protected double limitPower(double input) {
        if (input > MAX_FORWARD_SPEED)
            return MAX_FORWARD_SPEED;
        else if (input < MAX_REVERSE_SPEED)
            return MAX_REVERSE_SPEED;
        else return input;
    }
}
