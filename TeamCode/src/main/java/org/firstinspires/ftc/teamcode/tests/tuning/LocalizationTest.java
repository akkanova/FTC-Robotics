package org.firstinspires.ftc.teamcode.tests.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.misc.RoadRunnerLog;

/**
 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tuning/LocalizationTest.java">
 *      Copied from Road Runner quickstart  -  LocalizationTest.java
 * </a>. Modified for mecanum drive base testing only.
 */
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HardwareManager hardwareManager = new HardwareManager(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareManager, new Pose2d(0, 0, 0));

        waitForStart();

        while(opModeIsActive()) {
            drive.setDrivePowers(
                new PoseVelocity2d(
                    new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
                )
            );

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.currentPose.position.x);
            telemetry.addData("y", drive.currentPose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.currentPose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            RoadRunnerLog.drawRobot(packet.fieldOverlay(), drive.currentPose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}