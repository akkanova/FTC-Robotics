package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.all_purpose.HardwareManager;

/**
 * Base class for all Self-Driving scripts, a.k.a Autonomous.
 * Providing all the required tools to do precise movements.
 */
public abstract class SelfDriving extends LinearOpMode {
    protected final double WHEEL_CIRCUMFERENCE = Math.PI * 0.098; // M
    protected final int COUNTS_PER_MOTOR_REVOLUTION = 900;
    protected final double COUNTS_PER_METER =
            COUNTS_PER_MOTOR_REVOLUTION / WHEEL_CIRCUMFERENCE;

    protected final double WRIST_GEAR_RATIO = 0;
    protected final double POSITIONS_PER_SERVO_ROTATION = 0;
    protected final double POSITIONS_PER_SERVO_ANGLE = POSITIONS_PER_SERVO_ROTATION / 360;
    protected final double HEIGHT_OF_ARMPIT_JOINT_FROM_GROUND = 1; // IN

    protected HardwareManager hardwareManager;

    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------
    protected final double MOVEMENT_POWER = 0.5;
    protected final double TURN_POWER  = 0.3;

    //------------------------------------------------------------------------------------------------
    // Movement
    //------------------------------------------------------------------------------------------------
    protected void move(double metersDistance) {
        if (!opModeIsActive())
            return;

        hardwareManager.resetWheelCounts();
        hardwareManager.doToAllWheels((wheel) -> wheel.setPower(MOVEMENT_POWER));

        double totalCounts = COUNTS_PER_METER * metersDistance;
        while (opModeIsActive() && hardwareManager.getAverageWheelCounts() <= totalCounts) {
            idle();
        }

        hardwareManager.doToAllWheels((wheel) -> wheel.setPower(0));
    }

    //------------------------------------------------------------------------------------------------
    // Strafing
    //------------------------------------------------------------------------------------------------
    protected void strafe(double metersDistance) {
        // Do that..
    }

    //------------------------------------------------------------------------------------------------
    // Rotation
    //------------------------------------------------------------------------------------------------
    protected void rotate(double degreeAngle) {
        if (!opModeIsActive())
            return;

        hardwareManager.imu.resetYaw();
        double initialAngle = hardwareManager.getCurrentDegreeHeading();

        double motorOffset = degreeAngle > 0 ? 1 : -1;
        double leftPower = TURN_POWER * motorOffset;
        double rightPower = TURN_POWER * -motorOffset;

        hardwareManager.frontLeftWheel.setPower(leftPower);
        hardwareManager.frontRightWheel.setPower(rightPower);
        hardwareManager.backLeftWheel.setPower(leftPower);
        hardwareManager.backRightWheel.setPower(rightPower);

        while(opModeIsActive() && hasReachedDesiredAngle(initialAngle, degreeAngle)) {
            idle();
        }

        hardwareManager.doToAllWheels((wheel) -> wheel.setPower(0));
    }

    protected boolean hasReachedDesiredAngle(double initialAngle, double turnAngle) {
        double targetAngle = initialAngle - turnAngle;
        double currentAngle = hardwareManager.getCurrentDegreeHeading();

        return turnAngle > 0
                ? currentAngle > targetAngle
                : currentAngle < targetAngle;
    }

    //------------------------------------------------------------------------------------------------
    // Arm Controls
    //------------------------------------------------------------------------------------------------
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
          -an angle value is returned and added to the wrist joints 0 angle
        */


    protected void moveWristServos(double endPosition, double timeInMillisecondsForFinish){
        if (!opModeIsActive())
            return;

        double TopLeftArmServoP = hardwareManager.TopLeftArmServo.getPosition();
        double TopRightArmServoP = hardwareManager.TopRightArmServo.getPosition();

        double TopRightArmServoDestinationSlope = (TopRightArmServoP + endPosition * WRIST_GEAR_RATIO)
                                                    /timeInMillisecondsForFinish;
        double TopLeftArmServoDestinationSlope = (TopLeftArmServoP - endPosition * WRIST_GEAR_RATIO)
                                                    /timeInMillisecondsForFinish;

        ElapsedTime elapsedTime = new ElapsedTime();
        while(opModeIsActive() && elapsedTime.milliseconds() < timeInMillisecondsForFinish)
        {
            hardwareManager.TopLeftArmServo.setPosition(
                    TopLeftArmServoDestinationSlope * elapsedTime.milliseconds() + TopLeftArmServoP
            );

            hardwareManager.TopRightArmServo.setPosition(
                    TopRightArmServoDestinationSlope * elapsedTime.milliseconds() + TopRightArmServoP
            );

            /*Don't get all mixed up with my garble, mathematically this is simply akin to
              drawing a straight line between two points on a graph. In this case the two points being the
              current servo position and the desired position by the function.
             */
        }
    }
    protected void moveArmpitServos(int endPosition){
        if (!opModeIsActive())
            return;

        hardwareManager.BottomLeftArmMotor.setTargetPosition(endPosition);
        hardwareManager.BottomRightArmMotor.setTargetPosition(endPosition);

    }
    protected void changeWristPosByRange(double range){
        if (!opModeIsActive())
            return;

        double angleAtRange = Math.atan(range/ HEIGHT_OF_ARMPIT_JOINT_FROM_GROUND);
        double angleToPosition = angleAtRange * POSITIONS_PER_SERVO_ANGLE;
        moveWristServos(angleToPosition, 5000);
    }



    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        hardwareManager = new HardwareManager(hardwareMap);
        waitForStart();
        runAutonomous();
    }

    protected abstract void runAutonomous();
}
