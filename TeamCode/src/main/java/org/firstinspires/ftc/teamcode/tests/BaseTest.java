package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.misc.DashboardUtils;


/** Class extended by all test OpModes. */
public abstract class BaseTest extends LinearOpMode {
    protected Tools setup(Pose2d initialPose) {
        return new Tools(hardwareMap, telemetry, initialPose);
    }

    protected Tools setup() {
        return setup(new Pose2d(0, 0, 0));
    }

    protected void sendLocationTelemetryPacket(MecanumDrive drive) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        DashboardUtils.drawRobot(packet.fieldOverlay(), drive.currentPose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    protected static class Tools {
        public HardwareManager hardwareManager;
        public MultipleTelemetry telemetry;
        public MecanumDrive drive;

        public Tools(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialPose) {
            this.hardwareManager = new HardwareManager(hardwareMap);
            this.drive = new MecanumDrive(hardwareManager, initialPose);
            this.telemetry = new MultipleTelemetry(telemetry,
                FtcDashboard.getInstance().getTelemetry());
        }
    }
}
