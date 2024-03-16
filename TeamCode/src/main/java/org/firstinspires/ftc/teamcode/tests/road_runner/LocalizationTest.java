package org.firstinspires.ftc.teamcode.tests.road_runner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tests.TestTools;

/**
 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tuning/LocalizationTest.java">
 *      Copied from Road Runner quickstart  -  LocalizationTest.java
 * </a>. Modified for mecanum drive base testing only. Assumes that the robot is
 * initially placed at the exact center of the stage.
 */
public class LocalizationTest extends LinearOpMode {
    public static double INITIAL_X = 0;
    public static double INITIAL_Y = 0;
    public static double INITIAL_HEADING = 0;

    @Override
    public void runOpMode() {
        TestTools testTools = new TestTools(hardwareMap, telemetry,
            new Pose2d(INITIAL_X, INITIAL_Y, INITIAL_HEADING));

        waitForStart();

        while(opModeIsActive()) {
            testTools.drive.setDrivePowers(
                new PoseVelocity2d(
                    new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
                )
            );

            testTools.drive.updatePoseEstimate();
            testTools.sendPositionTelemetry();

            testTools.telemetry.addData("x", testTools.drive.currentPose.position.x);
            testTools.telemetry.addData("y", testTools.drive.currentPose.position.y);
            testTools.telemetry.addData("heading (deg)",
                Math.toDegrees(testTools.drive.currentPose.heading.toDouble())
            );

            testTools.telemetry.update();
        }
    }
}