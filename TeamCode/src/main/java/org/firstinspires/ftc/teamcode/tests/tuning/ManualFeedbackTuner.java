package org.firstinspires.ftc.teamcode.tests.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.tests.BaseTest;

public class ManualFeedbackTuner extends BaseTest {
    public static double distance = 64;

    @Override
    public void runOpMode() {
        initializeBaseDrive();
        waitForStart();

        if (drive.localizer instanceof ThreeWheelLocalizer &&
            GlobalConfig.ThreeWheelLocalizerConfig.leftParallelYTicks == 0 &&
            GlobalConfig.ThreeWheelLocalizerConfig.rightParallelYTicks == 0 &&
            GlobalConfig.ThreeWheelLocalizerConfig.perpendicularXTicks == 0) {
            throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }

        while (opModeIsActive()) {
            Actions.runBlocking(
                drive.getNewActionBuilder(new Pose2d(0, 0, 0))
                    .lineToX(distance)
                    .lineToX(0)
                    .build()
            );
        }
    }
}
