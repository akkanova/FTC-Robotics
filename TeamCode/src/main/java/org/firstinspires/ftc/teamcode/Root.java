package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * With the power of class inheritance we don't have to write the
 * code used in both "MainMovements" and "MainAutonomous" twice.
 * */
public abstract class Root extends OpMode {
    // "Intelligent 9-axis absolute orientation sensor"
    protected IMU imu;

    // Wheel Motors
    protected DcMotor frontLeftWheel;
    protected DcMotor frontRightWheel;
    protected DcMotor backLeftWheel;
    protected DcMotor backRightWheel;

    // Arm Servos
    protected CRServo bottomLeftArmServo;
    protected CRServo bottomRightArmServo;
    protected CRServo topLeftArmServo;
    protected CRServo topRightArmServo;

    /////////////////////////////////////////////////////////////////////////////////////
    // Hardware Preparations Code
    /////////////////////////////////////////////////////////////////////////////////////

    /** Reset the motor encoder so that it reads zero ticks at the current position */
    protected void resetWheelEncoders() {
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        // Option here
        // RUN_USING_ENCODER - use encoders to determine the velocity
        // RUN_WITHOUT_ENCODERS - it'll determine it's own velocity
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }

    /** Setup all 4 Motors driving each Mecanum wheels */
    protected void setupWheelMotors() {
        // Hardware Preparations

        // Rough Chassis Sketch, Utilizing 4WD
        // `frontLeft` 0| |   | |0 `frontRight`
        //              | |   | |
        //              | |= =| |
        // `backLeft`  0| |   | |0 `backRight`

        frontLeftWheel = hardwareMap.dcMotor.get("FrontLeftM");
        frontRightWheel = hardwareMap.dcMotor.get("FrontRightM");
        backLeftWheel = hardwareMap.dcMotor.get("BackLeftM");
        backRightWheel = hardwareMap.dcMotor.get("BackRightM");

        frontLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        doToAllWheels((wheel) -> wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
    }

    protected void setupArmHardware() {
        bottomLeftArmServo = hardwareMap.crservo.get("BottomLeftS");
        bottomRightArmServo = hardwareMap.crservo.get("BottomRightS");
        topLeftArmServo = hardwareMap.crservo.get("TopLeftS");
        topRightArmServo = hardwareMap.crservo.get("TopRightS");

        bottomLeftArmServo.setDirection(CRServo.Direction.FORWARD);
        bottomRightArmServo.setDirection(CRServo.Direction.REVERSE);
        topLeftArmServo.setDirection(CRServo.Direction.FORWARD);
        topRightArmServo.setDirection(CRServo.Direction.REVERSE);
    }

    // Required for autonomous turning
    protected void setup9AxisSensor() {
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters());
    }

    /**
     * Got tired of writing the same code for each of the wheels
     * Instead of doing this:
     *   frontLeftWheel.doSomething()
     *   frontRightWheel.doSomething();
     *   backLeftWheel.doSomething();
     *   backRightWheel.doSomething();
     *
     * you could instead do this instead:
     *  doToAllWheels((wheel) -> wheel.doSomething());
     * */
    protected void doToAllWheels(WheelCallback callback) {
        callback.run(frontLeftWheel);
        callback.run(frontRightWheel);
        callback.run(backLeftWheel);
        callback.run(backRightWheel);
    }

    interface WheelCallback { void run(DcMotor wheel); }

    /////////////////////////////////////////////////////////////////////////////////////
    // Telemetry Helper Code
    /////////////////////////////////////////////////////////////////////////////////////

    /** Tell the operator that all of the Hardware preparations are done. */
    protected void sendInitialTelemetry() {
        telemetry.addLine("Hardware Prepped. Press PLAY to start.");
        telemetry.update();
    }

    protected void appendTotalRuntime() {
        telemetry.addData("Status","Total Runtime %.3f s", getRuntime());
    }

    /** For development purposes only */
    protected void appendMotorDebugTelemetry() {
        telemetry.addData("FL Motor Position", frontLeftWheel.getCurrentPosition());
        telemetry.addData("FR Motor Position", frontRightWheel.getCurrentPosition());
        telemetry.addData("BL Motor Position", backLeftWheel.getCurrentPosition());
        telemetry.addData("BR Motor Position", backRightWheel.getCurrentPosition());
    }

    protected void appendServoDebugTelemetry() {
        telemetry.addData("TL CRServo Power", topLeftArmServo.getPower());
        telemetry.addData("TR CRServo Power", topRightArmServo.getPower());
        telemetry.addData("BL CRServo Power", bottomLeftArmServo.getPower());
        telemetry.addData("BR CRServo Power", bottomRightArmServo.getPower());
    }
}