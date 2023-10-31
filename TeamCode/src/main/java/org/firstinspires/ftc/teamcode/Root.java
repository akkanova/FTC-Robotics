package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * With the power of class inheritance we don't have to write the
 * code used in both "MainMovements" and "MainAutonomous" twice.
 * */
public abstract class Root extends OpMode {
    // Wheel Motors
    protected DcMotor frontLeftWheel;
    protected DcMotor frontRightWheel;
    protected DcMotor backLeftWheel;
    protected DcMotor backRightWheel;

    // Arm Servos
    protected Servo bottomLeftArmServo;
    protected Servo bottomRightArmServo;
    protected Servo topLeftArmServo;
    protected Servo topRightArmServo;

    /////////////////////////////////////////////////////////////////////////////////////
    // Hardware Preparations Code
    /////////////////////////////////////////////////////////////////////////////////////

    /** Setup the 4 wheel motors and reset encoders used for precision driving */
    protected void setupPreciseDcMotors() {
        setupDcMotors();

        // Reset the motor encoder so that it reads zero ticks at
        // the current position
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn the motor back on, basically required if you
        // use STOP_AND_RESET_ENCODER
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /** Setup all 4 Motors driving each Mecanum wheels */
    protected void setupDcMotors() {
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
    }

    protected void setupArmServos() {
        // Todo: Try running the servos as a Continuous Rotation Servo

        bottomLeftArmServo = hardwareMap.servo.get("BottomLeftS");
        bottomRightArmServo = hardwareMap.servo.get("BottomRightS");
        topLeftArmServo = hardwareMap.servo.get("TopLeftS");
        topRightArmServo = hardwareMap.servo.get("TopRightS");

        bottomLeftArmServo.setDirection(Servo.Direction.FORWARD);
        bottomRightArmServo.setDirection(Servo.Direction.REVERSE);
        topLeftArmServo.setDirection(Servo.Direction.FORWARD);
        topRightArmServo.setDirection(Servo.Direction.REVERSE);
    }

    /////////////////////////////////////////////////////////////////////////////////////
    // Telemetry Helper Code
    /////////////////////////////////////////////////////////////////////////////////////

    /** Tell the operator that all of the Hardware preparations are done. */
    protected void sendInitialTelemetry() {
        telemetry.addLine("Hardware Prepped. Press PLAY to start.");
        telemetry.update();
    }

    /** For development purposes only */
    protected void appendMotorDebugTelemetry() {
        telemetry.addData("FL Motor Power", frontLeftWheel.getPower());
        telemetry.addData("FR Motor Power", frontRightWheel.getPower());
        telemetry.addData("BL Motor Power", backLeftWheel.getPower());
        telemetry.addData("BR Motor Power", backRightWheel.getPower());
    }

    protected void appendServoDebugTelemetry() {
        telemetry.addData("TL Position", topLeftArmServo.getPosition());
        telemetry.addData("TR Position", topRightArmServo.getPosition());
        telemetry.addData("BL Position", bottomLeftArmServo.getPosition());
        telemetry.addData("BR Position", bottomRightArmServo.getPosition());
    }
}