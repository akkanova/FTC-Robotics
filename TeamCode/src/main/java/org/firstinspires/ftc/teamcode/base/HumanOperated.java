package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.all_purpose.HardwareManager;

/**
 * Base class for all human-operated scripts, a.k.a TeleOp.
 * Any inherited classes manipulates the given protected power
 * values of each motor and servos, and then calls `setHardwarePower`
 * which ensures that the values obeys the UPPER and LOWER limits,
 * before sending them to each hardware binding classes.
 *
 * (USER_INPUT) -> (Class extends HumanOperated) -> (Base Class HumanOperated) -> (Hardware Manager)
 *      |                   |                             |                              |
 *     \/                   |                             |                              `-> Magic Class that converts Java value to actual
 *   GamePad1 or            |                             |                                  voltage to be used by DcMotors and Servos.
 *   GamePad2               |                             |
 *                          |                             `> Ensures the desired power setting is within min and max
 *                          |                                 before sending it to the actual hardware binding class.
 *                         \/
 *                    Interprets the human control into whatever
 *                    control schema we decide. E.g Tank Control
 *                    Split Control, etc..
 */
public abstract class HumanOperated extends OpMode {
    protected HardwareManager hardwareManager;

    protected double frontLeftWheelP = 0;
    protected double frontRightWheelP = 0;
    protected double backLeftWheelP = 0;
    protected double backRightWheelP = 0;

    protected double bottomArmServoP = 0;
    protected double topArmServoP = 0;
    protected double clawServoP = 0;

    protected double droneReleaseServoP = 0;

    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------

    protected final double MOTOR_UPPER_POWER_LIMIT = 1;
    protected final double MOTOR_LOWER_POWER_LIMIT = -1;
    protected final double SERVO_UPPER_POWER_LIMIT = 0.8; // VEX Servos Actual Limitation
    protected final double SERVO_LOWER_POWER_LIMIT = -0.8; // VEX Servos Actual Limitation

    //------------------------------------------------------------------------------------------------
    // Defaults
    //------------------------------------------------------------------------------------------------

    // How much pressed the GamePad trigger has to be considered truthy.
    protected final double GP_TRIGGER_THRESHOLD = 0.5;
    protected final double SERVO_DELTA = 0.7;

    protected void useDefaultMovementControls() {
        // Allow for forward / backward movement command
        // to be receive from left and right joystick.
        double drive = gamepad1.left_stick_y != 0
                ? -gamepad1.left_stick_y
                : -gamepad1.right_stick_y;

        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        frontLeftWheelP  = drive - strafe + rotate;
        frontRightWheelP = drive - strafe - rotate;
        backLeftWheelP   = drive + strafe + rotate;
        backRightWheelP  = drive + strafe - rotate;
    }

    protected void useDefaultArmControl() {
        // Upper Arm
        if (gamepad1.left_bumper) {
            topArmServoP = SERVO_DELTA;
        } else if (gamepad1.left_trigger > GP_TRIGGER_THRESHOLD) {
            topArmServoP = -SERVO_DELTA;
        } else {
            topArmServoP = 0;
        }

        // Lower Arm
        // Side note: too heavy to maintain balance
        if (gamepad1.right_bumper) {
            bottomArmServoP = SERVO_DELTA;
        } else if (gamepad1.right_trigger > GP_TRIGGER_THRESHOLD) {
            bottomArmServoP = -SERVO_DELTA;
        } else {
            bottomArmServoP = 0;
        }

        // Claw
        if (gamepad1.dpad_up) {
            clawServoP = -SERVO_DELTA;
        } else if (gamepad1.dpad_down) {
            clawServoP = SERVO_DELTA;
        } else {
            clawServoP = 0;
        }
    }

    protected void useDefaultDroneLauncherControl() {
        droneReleaseServoP = gamepad1.start || gamepad2.start
                ? -SERVO_LOWER_POWER_LIMIT
                : 0;
    }

    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);
    }

    public void setHardwarePower() {
        hardwareManager.frontLeftWheel.setPower(limitMotorPower(frontLeftWheelP));
        hardwareManager.frontRightWheel.setPower(limitMotorPower(frontRightWheelP));
        hardwareManager.backLeftWheel.setPower(limitMotorPower(backLeftWheelP));
        hardwareManager.backRightWheel.setPower(limitMotorPower(backRightWheelP));

//        hardwareManager.topLeftArmServo.setPower(limitServoPower(topArmServoP));
//        hardwareManager.topRightArmServo.setPower(limitServoPower(topArmServoP));
//        hardwareManager.bottomLeftArmServo.setPower(limitServoPower(bottomArmServoP));
//        hardwareManager.bottomRightArmServo.setPower(limitServoPower(bottomArmServoP));
//
//        hardwareManager.droneReleaseServo.setPower(droneReleaseServoP);
//        hardwareManager.clawServo.setPower(clawServoP);
    }

    protected double limitMotorPower(double input) {
        return Range.clip(input, MOTOR_LOWER_POWER_LIMIT, MOTOR_UPPER_POWER_LIMIT);
    }

    protected double limitServoPower(double input) {
        return Range.clip(input, SERVO_LOWER_POWER_LIMIT, SERVO_UPPER_POWER_LIMIT);
    }
}
