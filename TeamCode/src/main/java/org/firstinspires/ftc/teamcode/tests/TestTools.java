package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.misc.DashboardUtils;

public class TestTools {
    public final HardwareManager hardwareManager;
    public final MultipleTelemetry telemetry;
    public final MecanumDrive drive;

    public TestTools(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialPose) {
        this.hardwareManager = new HardwareManager(hardwareMap);
        this.drive = new MecanumDrive(hardwareManager, initialPose);
        this.telemetry = new MultipleTelemetry(telemetry,
            FtcDashboard.getInstance().getTelemetry());

        RobotLog.d("Test Tools Initialized : ", System.nanoTime());
    }

    public TestTools(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, new Pose2d(0, 0, 0));
    }

    public void sendPositionTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        DashboardUtils.drawRobot(packet.fieldOverlay(), drive.currentPose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        RobotLog.d("Sent Position Telemetry to Dashboard (%.2f, %.2f, %.2f)",
            drive.currentPose.position.x,
            drive.currentPose.position.y,
            drive.currentPose.heading.toDouble()
        );
    }
}
