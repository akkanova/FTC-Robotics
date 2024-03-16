package org.firstinspires.ftc.teamcode.tests.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.tests.TestTools;

/**
 * Manually configure the initial and final position of a given servo.
 * Configure using FTC-Dashboard.
 * */
public class ServoPositionTuner extends LinearOpMode {
    public static String servoName = "servo";
    public static double position = 0;

    @Override
    public void runOpMode() {
        ServoImplEx servoImplEx = hardwareMap.get(ServoImplEx.class, servoName);
        waitForStart();

        while (opModeIsActive()) {
            servoImplEx.setPosition(position);
        }
    }
}
