package org.firstinspires.ftc.teamcode.tests.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;

/** Contains common code shared between some of the Tuning Test */
public abstract class BaseDebug extends LinearOpMode {
    protected HardwareManager hardwareManager;
    protected MecanumDrive mecanumDrive;

    /**
     * Initial the debug dependencies such as {@link HardwareManager},
     * {@link MecanumDrive} and {@link MultipleTelemetry}
     * */
    protected void initializeDependencies(Pose2d initialDrivePose) {
        if (hardwareMap == null)
            throw new RuntimeException("HardwareMap not initialized yet");

        hardwareManager = new HardwareManager(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareManager, initialDrivePose);
        telemetry = new MultipleTelemetry(telemetry,
            FtcDashboard.getInstance().getTelemetry());
    }

    /** Initializes dependencies but automatically sets the initial drive pose to 0, 0 */
    protected void initializeDependencies() {
        initializeDependencies(new Pose2d(0, 0, 0));
    }
}
