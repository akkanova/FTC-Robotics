package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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


    protected final double SECONDS_FOR_ONE_DEGREE_OF_MOTION = 0 / 180; // (How long it takes for the wrist to turn a 180)/(180 degrees)
    protected final double COUNTS_PER_SERVO_ANGLE = 1;

    protected final int COUNTS_PER_ARM_MOTOR_REVOLUTION = 1;
    protected final double COUNTS_PER_ANGLE =
            COUNTS_PER_ARM_MOTOR_REVOLUTION / 360.0; //DEGREES
    protected final double HEIGHT_OF_ARMPIT_JOINT_FROM_GROUND = 1; // IN

    protected final double WRIST_GEAR_RATIO = 1;
    protected final double ARMPIT_GEAR_RATIO = 1;

    protected HardwareManager hardwareManager;
    protected ElapsedTime elapsedTime;

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


    protected void moveWristServos(double endPositionAngle){
        if (!opModeIsActive())
            return;

        elapsedTime = new ElapsedTime();
        double finalEndPosition =
                (SECONDS_FOR_ONE_DEGREE_OF_MOTION) * endPositionAngle * WRIST_GEAR_RATIO * 1000;
        while(elapsedTime.milliseconds() < finalEndPosition)
        {
            hardwareManager.topLeftArmServo.setPower(MOVEMENT_POWER);
            hardwareManager.topRightArmServo.setPower(MOVEMENT_POWER);
        }
    }


    protected void moveArmpitServos(int endPositionAngle){
        if (!opModeIsActive())
            return;

        // A: look at the code for {@code move()}.. DcMotor.setTargetPosition is somewhat unreliable
        hardwareManager.resetBottomMotorCounts();
        hardwareManager.doToAllArmMotors((wheel)-> wheel.setPower(MOVEMENT_POWER));

        double totalCounts = COUNTS_PER_ANGLE * endPositionAngle * ARMPIT_GEAR_RATIO;
        while (opModeIsActive() && hardwareManager.getAverageBottomMotorCounts() <= totalCounts) {
            idle();
        }

        hardwareManager.doToAllArmMotors((wheel) -> wheel.setPower(0));
    }

    protected void changeWristPosByRange(double range){
        if (!opModeIsActive())
            return;

        double angleAtRange = Math.atan(range/ HEIGHT_OF_ARMPIT_JOINT_FROM_GROUND);
        double angleToPosition = angleAtRange * COUNTS_PER_SERVO_ANGLE;
        moveWristServos(angleToPosition);
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
