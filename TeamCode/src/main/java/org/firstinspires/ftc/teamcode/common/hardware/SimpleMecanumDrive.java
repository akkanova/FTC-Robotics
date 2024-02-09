package org.firstinspires.ftc.teamcode.common.hardware;

import static org.firstinspires.ftc.teamcode.common.HardwareManager.DriveBaseConstants.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.common.HardwareManager.DriveBaseConstants.TRACK_BASE;
import static org.firstinspires.ftc.teamcode.common.HardwareManager.DriveBaseConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.common.HardwareManager.DriveBaseConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.common.HardwareManager.DriveBaseConstants.kA;
import static org.firstinspires.ftc.teamcode.common.HardwareManager.DriveBaseConstants.kStatic;
import static org.firstinspires.ftc.teamcode.common.HardwareManager.DriveBaseConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.HardwareManager;

import java.util.ArrayList;
import java.util.List;

/**
 * Simple mecanum drive hardware implementation for REV hardware.
 * Loosely based on
 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/quickstart1/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/SampleMecanumDrive.java">
 *      Road Runner Quickstart - Sample Mecanum Drive
 * </a>
 */
public class SimpleMecanumDrive extends MecanumDrive {
    private final HardwareManager hardwareManager;

    private final int[] lastEncoderPositions;
    private final int[] lastEncoderVelocities;

    private TrajectoryFollower follower;

    /** Initialized by our HardwareManager */
    public SimpleMecanumDrive(HardwareManager hardwareManager) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_BASE, LATERAL_MULTIPLIER);
        this.hardwareManager = hardwareManager;

        // A: I don't think there's a mecanum drive with more than 4 wheels;
        this.lastEncoderPositions = new int[hardwareManager.wheelMotors.length];
        this.lastEncoderVelocities = new int[hardwareManager.wheelMotors.length];

        this.follower = new HolonomicPIDVAFollower();
    }

    //-----------------------------------------------------------------------------------
    // Road Runner Mecanum Drive Inheritance
    //-----------------------------------------------------------------------------------

    /** @return IMU yaw in radians */
    @Override
    protected double getRawExternalHeading() {
        return hardwareManager.imu
                .getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
    }

    /** @return wheel position measurements in inches */
    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int totalWheelMotors = hardwareManager.wheelMotors.length;
        List<Double> wheelPositions = new ArrayList<>(totalWheelMotors);

        for (int i = 0; i < totalWheelMotors; i++) {
            int encoderTicks = hardwareManager.wheelMotors[i].getCurrentPosition();
            wheelPositions.add(encoderTicksToInches(encoderTicks));
            lastEncoderPositions[i] = encoderTicks;
        }

        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        hardwareManager.getFrontLeftWheelMotor().setPower(v);
        hardwareManager.getFrontRightWheelMotor().setPower(v1);
        hardwareManager.getBackLeftWheelMotor().setPower(v2);
        hardwareManager.getBackRightWheelMotor().setPower(v3);
    }
}
