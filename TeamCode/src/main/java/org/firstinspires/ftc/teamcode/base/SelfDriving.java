package org.firstinspires.ftc.teamcode.base;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.all_purpose.HardwareManager;

/**
 * Base class for all Self-Driving scripts, a.k.a Autonomous.
 * Providing all the required tools to do precise movements.
 */
public abstract class SelfDriving extends LinearOpMode {
    protected HardwareManager hardwareManager;

    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------
    protected final Range<Double> MOVEMENT_POWER_RANGE = new Range<>(0.4, 1.0);
    protected final double TURN_POWER = 0.3;

    protected final long CLAW_OPEN_MS = 500;
    protected final long PAUSE_MS = 250;

    //------------------------------------------------------------------------------------------------
    // Movement
    //------------------------------------------------------------------------------------------------
    protected final double COUNTS_PER_REVOLUTION = 1010;
    protected final double TICKS_PER_METER = COUNTS_PER_REVOLUTION / (Math.PI * 0.098);
    protected final double MOVEMENT_MODIFIER = 1 / TICKS_PER_METER;
    protected final double TURNING_SLACK = 0; // Turning Error Compensation

    protected void move(double metersDistance) {
        if (!opModeIsActive())
            return;

        hardwareManager.resetWheelEncoders();

        double totalCounts = TICKS_PER_METER * metersDistance;
        double middleCount = totalCounts * 0.5 + MOVEMENT_POWER_RANGE.getLower();
        // + minimum power so it doesn't start at 0

        while (opModeIsActive() && (double) hardwareManager.getAverageWheelCounts() <= totalCounts) {
            // f(x) = -|x-(d1 - d2)/2| + (d1 + d2)/2 -- "upside-down absolute value
            // graph with the vertex between d1 and d2". Increase power based on this,
            // where x is distance traveled, d1 = 0, d2 total distance.. obviously cap
            // at max power (~1)..

            double wheelCounts = hardwareManager.getAverageWheelCounts();
            double power = -MOVEMENT_MODIFIER * Math.abs(wheelCounts - middleCount) + middleCount * MOVEMENT_MODIFIER;
            hardwareManager.doForAllWheels(wheel ->
                    wheel.setPower(MOVEMENT_POWER_RANGE.clamp(power)));

//            idle();
        }

        hardwareManager.doForAllWheels(wheel -> wheel.setPower(0));
        sleep(PAUSE_MS);
    }

    protected void rotate(double degreeAngle) {
        if (!opModeIsActive())
            return;

        hardwareManager.imu.resetYaw();
        double initialAngle = hardwareManager.getCurrentDegreeHeading();

        double motorOffset = degreeAngle > 0 ? 1 : -1;
        double leftPower = TURN_POWER * motorOffset;
        double rightPower = TURN_POWER * -motorOffset;

        hardwareManager.getFrontLeftWheel().setPower(leftPower);
        hardwareManager.getFrontRightWheel().setPower(rightPower);
        hardwareManager.getBackLeftWheel().setPower(leftPower);
        hardwareManager.getBackRightWheel().setPower(rightPower);

        while (opModeIsActive() && hasReachedDesiredAngle(initialAngle, degreeAngle)) {
            // idle();
        }

        hardwareManager.doForAllWheels(wheel -> wheel.setPower(0));
        sleep(PAUSE_MS);
    }

    protected boolean hasReachedDesiredAngle(double initialAngle, double turnAngle) {
        double targetAngle = initialAngle - turnAngle;
        double currentAngle = hardwareManager.getCurrentDegreeHeading();

        return turnAngle > 0
                ? currentAngle >= targetAngle - TURNING_SLACK
                : currentAngle <= targetAngle + TURNING_SLACK;
    }

    //------------------------------------------------------------------------------------------------
    // Arm Controls
    //------------------------------------------------------------------------------------------------
    protected final double SECONDS_FOR_ONE_DEGREE_OF_MOTION = 0.0 / 180; // (How long it takes for the wrist to turn a 180)/(180 degrees)
    protected final double COUNTS_PER_SERVO_ANGLE = 1;

    protected final double COUNTS_PER_ANGLE = COUNTS_PER_REVOLUTION / 360.0; // DEGREES
    protected final double HEIGHT_OF_ARMPIT_JOINT_FROM_GROUND = 1; // M

    protected final double WRIST_GEAR_RATIO = 1;
    protected final double ARMPIT_GEAR_RATIO = 1;
    protected final double ARM_MOVEMENT_POWER = 0.5;

    //The Arm, if possible by default, would be arranged in a rectangular shape
        /*
              [+] ---------- [+]
              |               |
              |               |
              |      <-range- N -range->
          =======

          [+]: joints
          N: claw

          -wrist joint is at a 0 degree position
          -range is the distance or point that the claw points at
          -an angle value is r  eturned and added to the wrist joints 0 angle
        */


    protected void moveWristServos(double endPositionAngle) {
        if (!opModeIsActive())
            return;

        double requiredTimeMS = (SECONDS_FOR_ONE_DEGREE_OF_MOTION) * endPositionAngle * WRIST_GEAR_RATIO * 1000;
        hardwareManager.topArmServo.setPower(ARM_MOVEMENT_POWER);

        ElapsedTime elapsedTime = new ElapsedTime();
        while (opModeIsActive() && elapsedTime.milliseconds() < requiredTimeMS) {
            idle();
        }

        hardwareManager.topArmServo.setPower(0);
    }

    protected void moveArmpitServos(int endPositionAngle){
        if (!opModeIsActive())
            return;

        hardwareManager.resetDcMotorEncoder(hardwareManager.bottomArmMotor);
        hardwareManager.bottomArmMotor.setPower(ARM_MOVEMENT_POWER);

        double totalCounts = COUNTS_PER_ANGLE * endPositionAngle * ARMPIT_GEAR_RATIO;
        while (opModeIsActive() && hardwareManager.bottomArmMotor.getCurrentPosition() <= totalCounts) {
            idle();
        }

        hardwareManager.bottomArmMotor.setPower(0);
    }

    protected void changeWristPosByRange(double range){
        if (!opModeIsActive())
            return;

        double angleAtRange = Math.atan(range / HEIGHT_OF_ARMPIT_JOINT_FROM_GROUND);
        double angleToPosition = angleAtRange * COUNTS_PER_SERVO_ANGLE;
        moveWristServos(angleToPosition);
    }

    protected void moveWristTillSeconds(double ms) {
        if (!opModeIsActive())
            return;

        ElapsedTime elapsedTime = new ElapsedTime();
        double requiredTimeMS = ms * WRIST_GEAR_RATIO;
        hardwareManager.topArmServo.setPower(ARM_MOVEMENT_POWER);

        while(opModeIsActive() && elapsedTime.milliseconds() < requiredTimeMS) {
            idle();
        }

        hardwareManager.topArmServo.setPower(0);
    }

    protected void openClaw() {
        if (!opModeIsActive())
            return;

        hardwareManager.clawServo.setPower(0.7);
        sleep(CLAW_OPEN_MS);
        hardwareManager.clawServo.setPower(0);
    }

    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------
    protected abstract void runAutonomous();

    @Override
    public void runOpMode() {
        hardwareManager = new HardwareManager(hardwareMap);
        waitForStart();
        runAutonomous();
    }
}
