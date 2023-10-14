package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "Basic Movements", group= "Linear OpMode")

/*
* This will serve as proof of concept for our robot, only containing the basic movements.
* Only covering Forward, Backwards, Side-ways Strafing, and Rotation.
*
* Use Hardware Configuration: `Config-2023`
*
* =================================IMPORTANT===================================
* Please ensure that the GamePad controller is plugged into the furthest port
* from the power switch. Diagram shown here:
*
*                 -------------------
*                 |  Blank          |
*                 |       Screen    |
*                 -------------------
*  Ethernet Port   ⮥  ⭡            ⮤ Power button Here
*                     |
*              ->   PLUG HERE!!  <-
*
* =================================IMPORTANT===================================
* */
public class BasicMovements extends LinearOpMode {
    // All Motors
    private DcMotor frontLeftM;
    private DcMotor frontRightM;
    private DcMotor backLeftM;
    private DcMotor backRightM;

    private final double MAX_REVERSE = -1.0;
    private final double MAX_FORWARD = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Preparations

        // Rough Chassis Sketch
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

        // Wait for the user to click the PLAY BUTTON
        waitForStart();
        ElapsedTime totalRuntime = new ElapsedTime();

        while(opModeIsActive()) {
            double stickPitch = -gamepad1.left_stick_y;
            double stickRoll = gamepad1.left_stick_x;

            double leftPower  = clampPower(stickPitch + stickRoll);
            double rightPower = clampPower(stickPitch - stickRoll);

            frontLeftM.setPower(leftPower);
            frontRightM.setPower(rightPower);
            backRightM.setPower(rightPower);
            backLeftM.setPower(leftPower);

            telemetry.addData("Status","Total Runtime %.3f s", totalRuntime.seconds());

            telemetry.addData("FL Motor Power", frontLeftM.getPower());
            telemetry.addData("FR Motor Power", frontRightM.getPower());
            telemetry.addData("BL Motor Power", backLeftM.getPower());
            telemetry.addData("BR Motor Power", backRightM.getPower());

            telemetry.update();
        }
    }

    /** Given the power, ensure that it doesn't exceed minimum and maximum */
    public double clampPower(double input) {
        if (input > MAX_FORWARD)
            return MAX_FORWARD;
        else if (input < MAX_REVERSE)
            return MAX_REVERSE;
        else return input;
    }
}
