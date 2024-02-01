package org.firstinspires.ftc.teamcode.base;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    protected final double COUNTS_PER_ELBOW_REVOLUTION = 1440;
    protected final double COUNTS_PER_ANGLE = COUNTS_PER_ELBOW_REVOLUTION / 360.0; // DEGREES
    protected final double ELBOW_MOTOR_POWER = 0.5;

    protected void moveElbowMotors(double endPositionAngle) {
        if (!opModeIsActive())
            return;

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

        while (opModeIsActive() && hardwareManager.elbowArmMotor.getCurrentPosition() < target_angle) {
            idle();
        }

        hardwareManager.elbowArmMotor.setPower(0);
    }

    protected void moveWristTillSeconds(double ms) {
        if (!opModeIsActive())
            return;

        ElapsedTime elapsedTime = new ElapsedTime();
        hardwareManager.elbowArmMotor.setPower(ELBOW_MOTOR_POWER);
        while(opModeIsActive() && elapsedTime.milliseconds() <= ms){
            idle();
        }
        hardwareManager.elbowArmMotor.setPower(0);
    }


    protected void openClaw() {
        if (!opModeIsActive())
            return;

        //hardwareManager.clawServo.setPower(0.7);
        //sleep(CLAW_OPEN_MS);
        //hardwareManager.clawServo.setPower(0);
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
