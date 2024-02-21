package org.firstinspires.ftc.teamcode.tests.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;

/** Contains common code shared between some of the Tuning Test */
public abstract class BaseTuningTest extends LinearOpMode {
    protected HardwareManager hardwareManager;
    protected MecanumDrive drive;

    /** Initialize ftc-dashboard telemetry, {@link MecanumDrive} and {@link HardwareManager} */
    protected void initializeRequirements() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardwareManager = new HardwareManager(hardwareMap);
        drive = new MecanumDrive(hardwareManager, new Pose2d(0, 0, 0));
    }
}
