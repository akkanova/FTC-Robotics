package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;

/** Class extended by all test OpModes. */
public abstract class BaseTest extends LinearOpMode {
    protected HardwareManager hardwareManager;
    protected MecanumDrive drive;

    /** Initializes the variable `drive` and `hardwareManager` */
    protected void initializeBaseDrive(Pose2d initialPose) {
        hardwareManager = new HardwareManager(hardwareMap);
        drive = new MecanumDrive(hardwareManager, initialPose);
    }

    /**
     * Initializes the variable `drive` and
     * `hardwareManager` with the default position
     * */
    protected void initializeBaseDrive() {
        initializeBaseDrive(new Pose2d(0, 0, 0));
    }

    /** Replaces `telemetry` with the FTC-Dashboard {@link MultipleTelemetry} */
    protected void initializeDashboardTelemetry() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
}
