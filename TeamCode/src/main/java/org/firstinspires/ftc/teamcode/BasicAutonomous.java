package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Basic Autonomous", group = "Autonomous")
/**
 * This will serve as a proof of concept for our really basic
 * autonomous mode, it will serve as a base to develop a library
 * to convert specific movements to motor rotation.
 *
 * The easiest way to implement autonomous is to measure time
 * given a certain action.
 * */

public class BasicAutonomous extends BasicLinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    // Percentages (0.50 = 50% of max speed)
    private final double MOVEMENT = 0.5;

    // Timings (It took the robot n amount of seconds to do these)
    private final double TURN_90 = 5;
    // Move by the total length of the robot
    private final double MOVE = 5;
    private final double STRAFE = 5;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Preparations
        setupDcMotors();

        // Wait for the user to click the PLAY BUTTON
        waitForStart();
        runtime.reset();
    }

    /**
     * direction = 1   Rotate Clockwise
     * direction = -1  Rotate Counter-Clockwise
     * */
    public void rotate90(int direction) {
        doThisFor(() -> {
            frontLeftM.setPower(MOVEMENT * direction);
            frontRightM.setPower(MOVEMENT * -direction);
            backLeftM.setPower(MOVEMENT * direction);
            backRightM.setPower(MOVEMENT * -direction);
        }, TURN_90);
    }

    /**
     * direction = 1   Forwards
     * direction = -1  Backwards
     * */
    public void move(int direction) {
        doThisFor(() -> {
            frontLeftM.setPower(MOVEMENT * direction);
            frontRightM.setPower(MOVEMENT * direction);
            backLeftM.setPower(MOVEMENT * direction);
            backRightM.setPower(MOVEMENT * direction);
        }, MOVE);
    }


    /**
     * direction = 1   Right
     * direction = -1  Left
     * */
    public void strafe(int direction) {
        doThisFor(() -> {
            frontLeftM.setPower(MOVEMENT * direction);
            frontRightM.setPower(MOVEMENT * -direction);
            backLeftM.setPower(MOVEMENT * -direction);
            backRightM.setPower(MOVEMENT * direction);
        }, STRAFE);
    }

    public void doThisFor(CallBack callBack, double seconds) {
        while(opModeIsActive() && runtime.seconds() < seconds) {
            callBack.run();
        }
    }
}

interface CallBack {
    void run();
}
