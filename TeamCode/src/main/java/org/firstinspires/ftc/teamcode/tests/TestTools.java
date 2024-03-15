package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;

public class TestTools {
    public final HardwareManager hardwareManager;
    public final MultipleTelemetry telemetry;
    public final MecanumDrive drive;

    public TestTools(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialPose) {
        this.hardwareManager = new HardwareManager(hardwareMap);
        this.drive = new MecanumDrive(hardwareManager, initialPose);
        this.telemetry = new MultipleTelemetry(telemetry,
            FtcDashboard.getInstance().getTelemetry());
    }

    public TestTools(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, new Pose2d(0, 0, 0));
    }
}
