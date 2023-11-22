package org.firstinspires.ftc.teamcode.all_purpose;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A specific class designed for setting up all of the robot's
 * hardware binding classes, and presiding over groups of them to
 * be able to do synchronized commands.
 */
public class HardwareManager {
    //------------------------------------------------------------------------------------------------
    // Wheels
    //------------------------------------------------------------------------------------------------
    public final DcMotor frontLeftWheel;
    public final DcMotor frontRightWheel;
    public final DcMotor backLeftWheel;
    public final DcMotor backRightWheel;

    /**
     * let `n` be return value;
     *      n < 0 = Motors went reversed.
     *      n > 0 = Motors went forward.
     */
    public double getAverageWheelCounts() {
        return (frontLeftWheel.getCurrentPosition() +
                frontRightWheel.getCurrentPosition() +
                backLeftWheel.getCurrentPosition() +
                backRightWheel.getCurrentPosition()) / 4.0;
    }

    public void resetWheelCounts() {
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }

    /**
     * For each wheel motor run that specific callback.
     * Converting this:
     *      frontLeftWheel.doTheSameCommand();
     *      frontRightWheel.doTheSameCommand();
     *      backLeftWheel.doTheSameCommand();
     *      backRightWheel.doTheSameCommand();
     *
     * To this:
     *      doToAllWheels((wheel) -> wheel.doTheSameCommand());
     */
    public void doToAllWheels(WheelCallback callback) {
        callback.run(frontLeftWheel);
        callback.run(frontRightWheel);
        callback.run(backLeftWheel);
        callback.run(backRightWheel);
    }

    public interface WheelCallback {
        void run(DcMotor motor);
    }

    //------------------------------------------------------------------------------------------------
    // Arm
    //------------------------------------------------------------------------------------------------
    public final CRServo topLeftArmServo;
    public final CRServo topRightArmServo;
    public final DcMotor bottomLeftArmMotor;
    public final DcMotor bottomRightArmMotor;

    /** Similar to {@code doToAllWheels()} but only for the bottom arm DcMotors. */
    public void doToAllArmMotors(WheelCallback callback) {
        callback.run(bottomLeftArmMotor);
        callback.run(bottomRightArmMotor);
    }
    public double getAverageBottomMotorCounts() {
        return (bottomLeftArmMotor.getCurrentPosition() +
                bottomRightArmMotor.getCurrentPosition()
                / 2.0);
    }
    public void resetBottomMotorCounts() {
        doToAllArmMotors((wheel) -> wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        doToAllArmMotors((wheel) -> wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }

    //------------------------------------------------------------------------------------------------
    // Sensors
    //------------------------------------------------------------------------------------------------
    public IMU imu;

    /**
     * Returns a normalized robot yaw orientation in Degrees (°)
     *
     *               <- FORWARD ->
     *                     0
     * LEFT  -90           +         90 RIGHT
     *                 -180/180
     *                 BACKWARD
     */
    public double getCurrentDegreeHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public HardwareManager(HardwareMap hardwareMap) {
        // Wheels
        frontLeftWheel = hardwareMap.dcMotor.get("FrontLeftM");
        frontRightWheel = hardwareMap.dcMotor.get("FrontRightM");
        backLeftWheel = hardwareMap.dcMotor.get("BackLeftM");
        backRightWheel = hardwareMap.dcMotor.get("BackRightM");

        frontLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        doToAllWheels((wheel) -> wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

        // Arm
        topLeftArmServo = hardwareMap.crservo.get("TopLeftS");
        topRightArmServo = hardwareMap.crservo.get("TopRightS");
        bottomLeftArmMotor = hardwareMap.dcMotor.get("BottomLeftM");
        bottomRightArmMotor = hardwareMap.dcMotor.get("BottomRightM");

        topLeftArmServo.setDirection(CRServo.Direction.FORWARD);
        topLeftArmServo.setDirection(CRServo.Direction.REVERSE);
        bottomLeftArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomRightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        doToAllArmMotors((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

        // Sensors
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();
    }
}
