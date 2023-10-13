package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name= "Main Movements", group= "Linear OpMode")

public class BasicMovements extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private final double MIN_MOTOR_POWER = -1.0;
    private final double MAX_MOTOR_POWER = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Hardware Prepared");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            double stickPitch = -gamepad1.left_stick_y;
            double stickRoll = gamepad1.left_stick_x;

            double leftPower  = Range.clip(stickPitch + stickRoll, MIN_MOTOR_POWER, MAX_MOTOR_POWER);
            double rightPower = Range.clip(stickPitch - stickRoll, MIN_MOTOR_POWER, MAX_MOTOR_POWER);

//
//            if (gamepad1.dpad_up) {
//                frontLeft.setPower(MAX_MOTOR_POWER);
//                frontRight.setPower(MAX_MOTOR_POWER);
//                backLeft.setPower(MAX_MOTOR_POWER);
//                backRight.setPower(MAX_MOTOR_POWER);
//
//            } else if (gamepad1.dpad_down) {
//                frontLeft.setPower(MIN_MOTOR_POWER);
//                frontRight.setPower(MIN_MOTOR_POWER);
//                backLeft.setPower(MIN_MOTOR_POWER);
//                backRight.setPower(MIN_MOTOR_POWER);
//
//            } else if (gamepad1.dpad_left) {
//                frontLeft.setPower(MAX_MOTOR_POWER);
//                frontRight.setPower(MIN_MOTOR_POWER);
//                backLeft.setPower(MAX_MOTOR_POWER);
//                backRight.setPower(MIN_MOTOR_POWER);
//
//            } else if (gamepad1.dpad_right) {
//                frontLeft.setPower(MIN_MOTOR_POWER);
//                frontRight.setPower(MAX_MOTOR_POWER);
//                backLeft.setPower(MIN_MOTOR_POWER);
//                backRight.setPower(MAX_MOTOR_POWER);
//
//            } else {
//                frontLeft.setPower(0);
//                frontRight.setPower(0);
//                backLeft.setPower(0);
//                backRight.setPower(0);
//            }

            frontLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);
            backLeft.setPower(leftPower);

            telemetry.addData("FL Motor Power", frontLeft.getPower());
            telemetry.addData("FR Motor Power", frontRight.getPower());
            telemetry.addData("BL Motor Power", backLeft.getPower());
            telemetry.addData("BR Motor Power", backRight.getPower());

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
