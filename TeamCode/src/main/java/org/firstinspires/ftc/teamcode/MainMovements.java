package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main Movements", group = "TeleOp")
/**
 * This will serve as the main movements control for our robot, to be used in
 * official competitions and training purposes.
 *
 * Use Hardware Configuration: `Config-2023`
 *
 * =================================IMPORTANT===================================
 * Please ensure that the GamePad controller is plugged into the furthest port
 * from the power switch. Diagram shown here:
 *
 *                 -------------------
 *                 |  Blank          |
 *                 |       Screen    |
 *                 -------------------
 *  Ethernet Port   ⮥  ⭡            ⮤ Power button Here
 *                     |
 *              ->   PLUG HERE!!  <-
 *
 * =================================IMPORTANT===================================
 *
 * Current GamePad Control Schema
 *    (D_PAD)                    (Button X, Y, B, A)  ═ ▶ None
 *     ║   (Left-Stick)    (Right-Stick)
 *     ║        ║               ╚ ▶ Forward, Backward, Rotate Left or Right;
 *     ▼        ╚ ▶ For Omnidirectional Strafing;
 *     None
 * */
public class MainMovements extends Root {
    // Movement Config
    // (0.50 => 50% of Max Speed,
    // -0.75 => 75% of Max Speed in the opposite direction)
    public final double MAX_REVERSE_SPEED = -1.00;
    public final double MAX_FORWARD_SPEED = 1.00;
    public final double ARM_SERVO_DELTA = 0.01;
    public final double GP_TRIGGER_THRESHOLD = 0.5;

    // temporary(?) variable for testing continuous servo use
    public double topServoPower = 0;
    public double bottomServoPower = -1;


    /** Runs once, before the operator presses PLAY */
    @Override
    public void init() {
        setupWheelMotors();
        setupArmHardware();
        sendInitialTelemetry();
    }

    /**
     *  Keeps running until operator presses STOP. Starts looping
     *  when operator presses PLAY
     * */
    @Override
    public void loop() {
        // Joystick value
        //      1 (y-value)
        //      ⭡
        // -1 ⭠ 0 ⭢ 1 (x-value)
        //      ⭣
        //     -1

        // If the left joy stick does not receive any up/down input, try reading
        // from the right joystick. Allowing forward & back controls to be
        // received from both left and right joysticks.
        double drive = -((gamepad1.left_stick_y != 0)
                ? gamepad1.left_stick_y
                : gamepad1.right_stick_y);

        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Percentage speed Capped at (MAX_REVERSE_SPEED and MAX_FORWARD_SPEED)
        double frontLeftP  = limitPower(drive + strafe + rotate);
        double frontRightP = limitPower(drive - strafe - rotate);
        double backLeftP   = limitPower(drive - strafe + rotate);
        double backRightP  = limitPower(drive + strafe - rotate);

        frontLeftWheel.setPower(frontLeftP);
        frontRightWheel.setPower(frontRightP);
        backLeftWheel.setPower(backLeftP);
        backRightWheel.setPower(backRightP);

        if (gamepad1.left_bumper) {
            topServoPower += ARM_SERVO_DELTA;
        } else if (gamepad1.left_trigger > GP_TRIGGER_THRESHOLD) {
            topServoPower -= ARM_SERVO_DELTA;
        }

        if (gamepad1.right_bumper) {
            bottomServoPower += ARM_SERVO_DELTA;
        } else if (gamepad1.right_trigger > GP_TRIGGER_THRESHOLD) {
            bottomServoPower -= ARM_SERVO_DELTA;
        }

        topServoPower    = limitPower(topServoPower);
        bottomServoPower = limitPower(bottomServoPower);

        // Danny's note: `setPower` acts more like `setPosition` of a traditional servo at the moment
        topLeftArmServo.setPower(topServoPower);
        topRightArmServo.setPower(topServoPower);
        bottomLeftArmServo.setPower(bottomServoPower);
        bottomRightArmServo.setPower(bottomServoPower);

        // Replace with something else in official competitions
        appendMotorDebugTelemetry();
        appendServoDebugTelemetry();
        telemetry.update();
    }

    public double limitPower(double input) {
        if (input > MAX_FORWARD_SPEED)
            return MAX_FORWARD_SPEED;
        else if (input < MAX_REVERSE_SPEED)
            return MAX_REVERSE_SPEED;
        else return input;
    }
}
