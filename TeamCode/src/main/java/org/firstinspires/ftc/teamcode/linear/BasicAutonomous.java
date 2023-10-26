package org.firstinspires.ftc.teamcode.linear;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Basic Autonomous", group = "Autonomous")
/**
 * =================================IMPORTANT===================================
 *
 * This has been archived, as encoders are preferred over timing based movements
 * for accuracy and redundancy.
 *
 * =================================IMPORTANT===================================
 *
 *
 * This will serve as a proof of concept for our really basic
 * autonomous mode code, whilst also serving as a base to develop a library
 * to convert specific movements to motor rotation.
 *
 * The easiest way to implement autonomous is to measure time
 * for certain action.
 * */

public class BasicAutonomous extends BasicLinearOpMode {
    // Percentages (0.50 = 50% of max speed)
    private final double MOVEMENT = 0.5;

    // Timings (It took the robot n amount of seconds to do these)
    private final double TURN_90 = 5;
    private final double LENGTH_TRAVERSAL_TIME = 5;
    private final double STRAFE_TRAVERSAL_TIME = 5;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Preparations
        setupDcMotors();

        // Wait for the user to click the PLAY BUTTON
        waitForStart();

        // Write autonomous traversal code ...
    }

    /**
     * direction = 1   Rotate Clockwise
     * direction = -1  Rotate Counter-Clockwise
     * */
    public void rotate90(int direction) {
        runMotors(
            direction, -direction,
            direction, -direction,
            TURN_90
        );
    }

    /**
     * direction = 1   Forwards
     * direction = -1  Backwards
     * */
    public void move(int direction) {
        runMotors(
            direction, direction,
            direction, direction,
            LENGTH_TRAVERSAL_TIME
        );
    }

    /**
     * direction = 1   Right
     * direction = -1  Left
     * */
    public void strafe(int direction) {
        runMotors(
            direction, -direction,
            -direction, direction,
            STRAFE_TRAVERSAL_TIME
        );
    }

    /**
     * Run each motors with the following direction
     * where (+1 = Forward, -1 = Reverse) for
     * n amount of seconds.
     *  */
    public void runMotors(
        int frontLeftDirection,
        int frontRightDirection,
        int backLeftDirection,
        int backRightDirection,
        double totalRuntimeSeconds
    ) {
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && runtime.seconds() < totalRuntimeSeconds) {
            frontLeftM.setPower(MOVEMENT * frontLeftDirection);
            frontRightM.setPower(MOVEMENT * frontRightDirection);
            backLeftM.setPower(MOVEMENT * backLeftDirection);
            backRightM.setPower(MOVEMENT * backRightDirection);

            sendMotorsTelemetry();
        }
    }
}