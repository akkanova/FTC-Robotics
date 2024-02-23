package org.firstinspires.ftc.teamcode.tests.debug;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.common.hardware.localizers.MecanumLocalizer;

/**
 *  Using
 *  <a href="http://192.168.43.1:8080/dash">
 *      FTC-Dashboard
 * </a>
 * see the real time velocities of each drive base wheel
 * motors graphed over time.
 * */
public class WheelVelocitiesDebug extends BaseDebug {
    @Override
    public void runOpMode() {
        initializeDependencies();
        if (!(mecanumDrive.localizer instanceof MecanumLocalizer))
            throw new RuntimeException("Drive-base must use the built-in encoders to utilize this OpMode");

        MecanumLocalizer localizer = (MecanumLocalizer) mecanumDrive.localizer;
        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive.setDrivePowers(
                new PoseVelocity2d(
                    new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
                )
            );

            telemetry
                .addData("Front Left Vel (in/s)",  localizer.frontLeftWheelEncoder.getPositionAndVelocity().velocity)
                .addData("Front Right Vel (in/s)", localizer.frontRightWheelEncoder.getPositionAndVelocity().velocity)
                .addData("Back Left Vel (in/s)",   localizer.backLeftWheelEncoder.getPositionAndVelocity().velocity)
                .addData("Back Right Vel (in/s)",  localizer.backRightWheelEncoder.getPositionAndVelocity().velocity);
        }
    }
}
