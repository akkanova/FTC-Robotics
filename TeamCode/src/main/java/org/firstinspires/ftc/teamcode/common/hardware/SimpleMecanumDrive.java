package org.firstinspires.ftc.teamcode.common.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Configuration;
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

    private final ArrayList<Integer> lastEncoderPositions;
    private final ArrayList<Integer> lastEncoderVelocities;

    private final TrajectoryFollower follower;

    /** Initialized by our HardwareManager */
    public SimpleMecanumDrive(HardwareManager hardwareManager) {
        super(
            Configuration.FeedForwardPID.kV,
            Configuration.FeedForwardPID.kA,
            Configuration.FeedForwardPID.kStatic,
            Configuration.DriveBaseConstants.TRACK_WIDTH,
            Configuration.DriveBaseConstants.TRACK_BASE,
            Configuration.MecanumDriveBaseConfig.LATERAL_MULTIPLIER
        );

        this.hardwareManager = hardwareManager;
        this.lastEncoderPositions = new ArrayList<>();
        this.lastEncoderVelocities = new ArrayList<>();
        this.follower = new HolonomicPIDVAFollower(
            Configuration.MecanumDriveBaseConfig.TRANSLATIONAL_PID,
            Configuration.MecanumDriveBaseConfig.TRANSLATIONAL_PID,
            Configuration.MecanumDriveBaseConfig.HEADING_PID,
            new Pose2d(0.5, 0.5, Math.toRadians(5)),
            0.5
        );
    }

    //-----------------------------------------------------------------------------------
    // Road Runner MecanumDrive Inheritance
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
        List<Double> wheelPositions = new ArrayList<>(hardwareManager.wheelMotors.length);
        lastEncoderPositions.clear();

        for (DcMotorEx wheelMotor : hardwareManager.wheelMotors) {
            int encoderTicks = wheelMotor.getCurrentPosition();
            wheelPositions.add(Configuration.DriveBaseConstants.encoderTicksToInches(encoderTicks));
            lastEncoderPositions.add(encoderTicks);
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
