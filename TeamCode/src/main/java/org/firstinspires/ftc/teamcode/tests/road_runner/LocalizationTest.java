package org.firstinspires.ftc.teamcode.tests.road_runner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.misc.DashboardUtils;
import org.firstinspires.ftc.teamcode.tests.BaseTest;

/**
 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tuning/LocalizationTest.java">
 *      Copied from Road Runner quickstart  -  LocalizationTest.java
 * </a>. Modified for mecanum drive base testing only. Assumes that the robot is
 * initially placed at the exact center of the stage.
 */
public class LocalizationTest extends BaseTest {
    public static double INITIAL_X = 0;
    public static double INITIAL_Y = 0;
    public static double INITIAL_HEADING = 0;

    @Override
    public void runOpMode() {
        Tools tools = setup();
        waitForStart();

        while(opModeIsActive()) {
            tools.drive.setDrivePowers(
                new PoseVelocity2d(
                    new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
                )
            );

            tools.drive.updatePoseEstimate();

            telemetry.addData("x", tools.drive.currentPose.position.x);
            telemetry.addData("y", tools.drive.currentPose.position.y);
            telemetry.addData("heading (deg)",
                Math.toDegrees(tools.drive.currentPose.heading.toDouble()));

            telemetry.update();
            sendLocationTelemetryPacket(tools.drive);
        }
    }
}