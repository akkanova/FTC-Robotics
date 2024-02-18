package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;

import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.MecanumLocalizer;

/**
 * Road Runner drive base implementation for
 * an all mecanum wheels assembly.

 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java">
 *     Based on Road Runner quickstart  -  MecanumDrive.java
 * </a>
 * */
public final class MecanumDrive {
    /** The localizer it's using to know where it is relative to the field */
    public final Localizer localizer;
    /** It's current known position in the field */
    public Pose2d pose2d;

    private final HardwareManager hardwareManager;

    /**
     * Create an instance of MecanumDrive that utilizes provided localizer.
     * */
    public MecanumDrive(
        HardwareManager hardwareManager,
        Pose2d initialPose,
        Localizer localizer
    ) {
        this.hardwareManager = hardwareManager;
        this.localizer = localizer;
        this.pose2d = initialPose;
    }

    /**
     * Create an instance of MecanumDrive that utilizes the default wheel
     * encoders for localization.
     * */
    public MecanumDrive(HardwareManager hardwareManager, Pose2d initialPose) {
        this(hardwareManager, initialPose, new MecanumLocalizer(hardwareManager));
    }

    /**
     * Sets the calculated necessary individual wheel power to achieve the
     * inputted position velocity.
     * */
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVelocities = new MecanumKinematics(1)
            .inverse(PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMagnitude = 1;
        for (DualNum<Time> power : wheelVelocities.all()) {
            maxPowerMagnitude = Math.max(maxPowerMagnitude, power.value());
        }

        hardwareManager.getFrontLeftWheelMotor().setPower(
            wheelVelocities.leftFront.get(0) / maxPowerMagnitude);
        hardwareManager.getFrontRightWheelMotor().setPower(
            wheelVelocities.rightFront.get(0) / maxPowerMagnitude);
        hardwareManager.getBackLeftWheelMotor().setPower(
            wheelVelocities.leftBack.get(0) / maxPowerMagnitude);
        hardwareManager.getBackRightWheelMotor().setPower(
            wheelVelocities.rightBack.get(0) / maxPowerMagnitude);
    }
}
