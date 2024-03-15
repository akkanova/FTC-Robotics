package org.firstinspires.ftc.teamcode.operated;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.misc.Lazy;

/** Base class implemented by all Tele-Op */
public abstract class BaseOperated extends OpMode {
    protected HardwareManager hardwareManager;
    protected MecanumDrive drive;

    protected  Lazy<GamepadEx> lazyGamepadEx1;
    protected  Lazy<GamepadEx> lazyGamepadEx2;

    /** Child should not override this */
    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);
        drive = new MecanumDrive(hardwareManager, getInitialPose());

        lazyGamepadEx1 = new Lazy<>(() -> new GamepadEx(gamepad1));
        lazyGamepadEx2 = new Lazy<>(() -> new GamepadEx(gamepad2));
    }

    /** Child should not override this */
    @Override
    public void loop() {
        if (lazyGamepadEx1.isInitialized())
            lazyGamepadEx1.get().sync();
        if (lazyGamepadEx2.isInitialized())
            lazyGamepadEx2.get().sync();

        drive.setDrivePowers(getDriveControls());
    }

    protected Pose2d getInitialPose() {
        return new Pose2d(0, 0, 0);
    }

    protected abstract PoseVelocity2d getDriveControls();
}
