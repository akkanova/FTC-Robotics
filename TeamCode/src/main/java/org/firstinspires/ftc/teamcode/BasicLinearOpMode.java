package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * With the power of class inheritance we don't have to write the
 * code used in both "BasicAutonomous" and "BasicMovements" twice.
 * */
public abstract class BasicLinearOpMode extends LinearOpMode {
    // All Motors
    protected DcMotor frontLeftM;
    protected DcMotor frontRightM;
    protected DcMotor backLeftM;
    protected DcMotor backRightM;

    protected void setupDcMotors() {
        // Hardware Preparations

        // Rough Chassis Sketch, Utilizing 4WD
        // `frontLeft` 0| |   | |0 `frontRight`
        //              | |   | |
        //              | |= =| |
        // `backLeft`  0| |   | |0 `backRight`

        frontLeftM  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightM = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftM   = hardwareMap.get(DcMotor.class, "backLeft");
        backRightM  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeftM.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightM.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftM.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightM.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Hardware Prepared");
        telemetry.addLine("Press the PLAY button to start.");
        telemetry.update();
    }

    /** Append Motors' data to telemetry and send data. */
    protected void sendMotorsTelemetry() {
        telemetry.addData("FL Motor Power", frontLeftM.getPower());
        telemetry.addData("FR Motor Power", frontRightM.getPower());
        telemetry.addData("BL Motor Power", backLeftM.getPower());
        telemetry.addData("BR Motor Power", backRightM.getPower());

        telemetry.update();
    }
}