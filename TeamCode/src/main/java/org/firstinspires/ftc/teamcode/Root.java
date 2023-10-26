package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * With the power of class inheritance we don't have to write the
 * code used in both "MainMovements" and "MainAutonomous" twice.
 * */
public abstract class Root extends OpMode {
    protected DcMotor frontLeftM;
    protected DcMotor frontRightM;
    protected DcMotor backLeftM;
    protected DcMotor backRightM;

    /////////////////////////////////////////////////////////////////////////////////////
    // Hardware Preparations Code
    /////////////////////////////////////////////////////////////////////////////////////

    /** Setup the 4 wheel motors and reset encoders used for precision driving */
    protected void setupPreciseDcMotors() {
        setupDcMotors();

        // Reset the motor encoder so that it reads zero ticks at
        // the current position
        frontLeftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn the motor back on, basically required if you
        // use STOP_AND_RESET_ENCODER
        frontLeftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Setup all 4 Motors driving each Mecanum wheels */
    protected void setupDcMotors() {
        // Hardware Preparations

        // Rough Chassis Sketch, Utilizing 4WD
        // `frontLeft` 0| |   | |0 `frontRight`
        //              | |   | |
        //              | |= =| |
        // `backLeft`  0| |   | |0 `backRight`

        frontLeftM  = hardwareMap.dcMotor.get("frontLeft");
        frontRightM = hardwareMap.dcMotor.get("frontRight");
        backLeftM   = hardwareMap.dcMotor.get("backLeft");
        backRightM  = hardwareMap.dcMotor.get("backRight");

        frontLeftM.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightM.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftM.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightM.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    protected void setupArmServos() {
        // Write code for that when it's prepared.
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
    protected void sendMotorDebugTelemetry() {
        telemetry.addData("FL Motor Power", frontLeftM.getPower());
        telemetry.addData("FR Motor Power", frontRightM.getPower());
        telemetry.addData("BL Motor Power", backLeftM.getPower());
        telemetry.addData("BR Motor Power", backRightM.getPower());

        telemetry.update();
    }
}