package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Basic Movements", group = "TeleOp")

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
public class BasicMovements extends BasicLinearOpMode {
    // Configurable percentage of speed. (0.5 = 50% of max forward speed)
    private final double MAX_REVERSE = -1.0;
    private final double MAX_FORWARD = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Preparations
        setupDcMotors();

        // Wait for the user to click the PLAY BUTTON
        waitForStart();
        ElapsedTime totalRuntime = new ElapsedTime();

        // GamePad Control Schema
        //  (D_PAD)                    (Button X, Y, B, A)  ═ ▶  Presets for autonomous (Not Included Here)
        //   ║   (Left-Stick)    (Right-Stick)
        //   ║        ║               ╚ ▶ Forward, Backward, Rotate Left or Right;
        //   ▼        ╚ ▶ For Omnidirectional Strafing;
        //   None

        while(opModeIsActive()) {
            // Joystick value
            //      1 (y-value)
            //      ⭡
            // -1 ⭠ 0 ⭢ 1 (x-value)
            //      ⭣
            //     -1

            // Allow to drive from left stick and right stick
            double drive = -((gamepad1.left_stick_y != 0)
                    ? gamepad1.left_stick_y
                    : gamepad1.right_stick_y);

            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Percentage Powers Capped at (MAX_REVERSE and MAX_FORWARD)
            double frontLeftP  = clampPower(drive + strafe + rotate);
            double frontRightP = clampPower(drive - strafe - rotate);
            double backLeftP   = clampPower(drive - strafe + rotate);
            double backRightP  = clampPower(drive + strafe - rotate);

            frontLeftM.setPower(frontLeftP);
            frontRightM.setPower(frontRightP);
            backLeftM.setPower(backLeftP);
            backRightM.setPower(backRightP);


            // DEBUG DATA
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