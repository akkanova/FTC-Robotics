package org.firstinspires.ftc.teamcode.tests.road_runner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.tests.TestTools;

public class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() {
        TestTools testTools = new TestTools(hardwareMap, telemetry);
        waitForStart();

        if (testTools.drive.localizer instanceof ThreeWheelLocalizer &&
            GlobalConfig.ThreeWheelLocalizerConfig.leftParallelYTicks == 0 &&
            GlobalConfig.ThreeWheelLocalizerConfig.rightParallelYTicks == 0 &&
            GlobalConfig.ThreeWheelLocalizerConfig.perpendicularXTicks == 0) {
            throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }

        while (opModeIsActive()) {
            Actions.runBlocking(
                testTools.drive.getNewActionBuilder(new Pose2d(0, 0, 0))
                    .lineToX(DISTANCE)
                    .lineToX(0)
                    .build()
            );
        }
    }
}
