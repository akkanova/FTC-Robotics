package org.firstinspires.ftc.teamcode.tests.debug;


import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.tests.BaseTest;

/** To test field-centric controls..*/
public class FieldCentricDebug extends BaseTest {
    @Override
    public void runOpMode() {
        HardwareManager hardwareManager = getHardwareManager();
        MecanumDrive mecanumDrive = getMecanumDrive();

        waitForStart();
//        Gamepad

        while (opModeIsActive()) {
            mecanumDrive.updatePoseEstimate();
//            mecanumDrive.setDrivePowers(
//                new Pose2d(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x
//                )
//            );
        }
    }
}
