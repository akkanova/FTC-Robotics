package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Range;

import org.firstinspires.ftc.teamcode.all_purpose.HardwareManager;
import org.firstinspires.ftc.teamcode.all_purpose.Misc;

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

    protected double frontLeftWheelPower;
    protected double frontRightWheelPower;
    protected double backLeftWheelPower;
    protected double backRightWheelPower;

    protected double bottomArmMotorPower;
    protected double topArmServoPower;
    protected double clawServoPower;

    protected double liftMotorPower;

    protected boolean unhook;

    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------
    // Power range [MINIMUM, MAXIMUM]
    protected static final Range<Double> WHEELS_POWER_RANGE    = new Range<>(-1.0, 1.0);
    protected static final Range<Double> ARM_MOTOR_POWER_RANGE = new Range<>(-1.0, 1.0);
    protected static final Range<Double> ARM_SERVO_POWER_RANGE = new Range<>(-0.8, 0.8);
    protected static final Range<Double> LIFT_POWER_RANGE      = new Range<>(-1.0, 0.5);
    protected static final double LAUNCHER_BASE_POSITION = 0.5;

    //------------------------------------------------------------------------------------------------
    // Defaults
    //------------------------------------------------------------------------------------------------
    protected static final double GAME_PAD_TRIGGER_THRESHOLD = 0.5;
    protected static final double LIFT_POWER_DELTA = 1.0;
    protected static final double ARM_MOTOR_DELTA = 1.0;
    protected static final double ARM_SERVO_DELTA = 1.0;

    protected void useDefaultMovementControls() {
        double strafe = Misc.easeWithCubic(gamepad1.left_stick_x);
        double rotate = Misc.easeWithCubic(gamepad1.right_stick_x);
        double drive  = Misc.easeWithCubic(
                gamepad1.left_stick_y != 0
                        ? -gamepad1.left_stick_y
                        : -gamepad1.right_stick_y
        );

        double heading = Math.atan2(drive, strafe);
        double power = Math.hypot(strafe, drive);

        double sin = Math.sin(heading - Math.PI / 4);
        double cos = Math.cos(heading - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftWheelPower  = power * cos / max + rotate;
        frontRightWheelPower = power * sin / max - rotate;
        backLeftWheelPower   = power * sin / max + rotate;
        backRightWheelPower  = power * cos / max - rotate;

        if ((power + Math.abs(rotate)) > 1) {
            frontLeftWheelPower  /= power + rotate;
            frontRightWheelPower /= power + rotate;
            backLeftWheelPower   /= power + rotate;
            backRightWheelPower  /= power + rotate;
        }
    }

    protected void useDefaultArmControls() {
        bottomArmMotorPower = 0;
        topArmServoPower = 0;
        clawServoPower = 0;

        // Bottom Arm
        if (gamepad1.left_bumper)
            bottomArmMotorPower = ARM_MOTOR_DELTA;
        else if (gamepad1.left_trigger > GAME_PAD_TRIGGER_THRESHOLD)
            bottomArmMotorPower = -ARM_MOTOR_DELTA;

        // Top Arm
        if (gamepad1.right_bumper)
            topArmServoPower = ARM_SERVO_DELTA;
        else if (gamepad1.right_trigger > GAME_PAD_TRIGGER_THRESHOLD)
            topArmServoPower = -ARM_SERVO_DELTA;

        // Claw
        if (gamepad1.dpad_right)
            clawServoPower = ARM_SERVO_DELTA;
        else if (gamepad1.dpad_left)
            clawServoPower = -ARM_SERVO_DELTA;
    }

    protected void useDefaultLiftControls() {
        liftMotorPower = 0;

        if (gamepad1.dpad_up || gamepad2.dpad_up)
            liftMotorPower = LIFT_POWER_DELTA;
        else if (gamepad1.dpad_down || gamepad2.dpad_down)
            liftMotorPower = -LIFT_POWER_DELTA;
    }

    protected void useDefaultDroneLauncherControls() {
        unhook = gamepad1.start || gamepad2.start;
    }

    //------------------------------------------------------------------------------------------------
    // nEw ArM cONtrOls (experimental)
    //------------------------------------------------------------------------------------------------
    protected final double COUNTS_PER_ELBOW_REVOLUTION = 2880;
    protected final double COUNTS_PER_ANGLE = COUNTS_PER_ELBOW_REVOLUTION / 360.0; // DEGREES
    protected final double ELBOW_MOTOR_POWER = 0.5;
    protected final double WRIST_SERVO_POWER = 0.5;
    protected boolean isUp = false;
    protected void moveElbowMotors(double endPositionAngle) {
        /*
        * This is a copy of the moveElbowMotors() from selfDriving with a very very
        * experimental way of changing the direction of the motor
        * */
        if(endPositionAngle >= 0) {
            hardwareManager.elbowArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            hardwareManager.elbowArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        double total_counts = COUNTS_PER_ANGLE * endPositionAngle; // Angle of movement desired
        double motor_position = hardwareManager.elbowArmMotor.getCurrentPosition(); // Elbow's current position
        double target_angle = (endPositionAngle >= 0)
                ? (motor_position + total_counts)
                : (motor_position - total_counts); // The desired end position of the motors angle

        hardwareManager.elbowArmMotor.setPower(ELBOW_MOTOR_POWER);
        while (hardwareManager.elbowArmMotor.getCurrentPosition() <= target_angle) {
            //idle()
        }

        hardwareManager.elbowArmMotor.setPower(0);
    }
    protected void useSimpleArmControls(){
        if (!hardwareManager.elbowArmMotor.isBusy()) {
            if(gamepad1.x){
                if(!isUp){
                    moveElbowMotors(-90);
                    isUp = true;
                    // if x is pressed then arm moves down by 45 degrees
                }else{
                    moveElbowMotors(90);
                    isUp = false;
                    // if y is pressed then arm moves up by 45 degrees
                }
            }
        }
        telemetry.addData("IS UP: ", isUp);
        telemetry.update();

        //--- Claw controls ---
        if(gamepad1.a){
            moveClawServos(true);
        } else {
            hardwareManager.clawServoLeft.setPosition(0);
        }
        if(gamepad1.b){
            moveClawServos(false);
        } else {
            hardwareManager.clawServoRight.setPosition(0);
        }
    }

    //---- Wrist Controls ----

    protected void moveClawServos(boolean isServoLeft){
        if(isServoLeft){
            hardwareManager.clawServoLeft.setPosition(0.5);
        } else {
            hardwareManager.clawServoRight.setPosition(0.5);
        }
    }

    protected void wristRotation(){

    }


    //------------------------------------------------------------------------------------------------
    // Debug
    //------------------------------------------------------------------------------------------------

    protected void checkSpeedOfEachWheel(int currentTime, int changeInSeconds){
        if(currentTime % changeInSeconds == 0){
            //Re-add check speed functionality
        }
    }

    public void sleep(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e){}
    }

    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------

    protected abstract void processUserInput();

    @Override
    public void loop() {
        processUserInput();

        hardwareManager.getFrontLeftWheel().setPower(WHEELS_POWER_RANGE.clamp(frontLeftWheelPower));
        hardwareManager.getFrontRightWheel().setPower(WHEELS_POWER_RANGE.clamp(frontRightWheelPower));
        hardwareManager.getBackLeftWheel().setPower(WHEELS_POWER_RANGE.clamp(backLeftWheelPower));
        hardwareManager.getBackRightWheel().setPower(WHEELS_POWER_RANGE.clamp(backRightWheelPower));

        hardwareManager.liftMotor.setPower(LIFT_POWER_RANGE.clamp(liftMotorPower));

        //hardwareManager.droneLauncherBase.setPosition(0);
        //hardwareManager.droneLauncherHook.setPosition(0);

        //if (unhook)
            //hardwareManager.droneLauncherHook.setPosition(1);
    }

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);
    }

    @Override
    public void start() {
        //hardwareManager.droneLauncherBase.setPosition(LAUNCHER_BASE_POSITION);
        hardwareManager.clawServoLeft.setPosition(0);
        hardwareManager.clawServoRight.setPosition(0);
    }
}
