package org.firstinspires.ftc.teamcode.common.hardware.localizers;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.HardwareManager;

/**
 * Localizer that utilizes the built-in encoders within each of the 4-motors in a mecanum
 * drive-base. Due to unavoidable wheel slips during high speed maneuvers, any dead wheel
 * localizer is preferred over this.

 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java#L123">
 *      Copied from Road Runner quickstart  -  MecanumDrive.java:123
 * </a>
 */
public class MecanumLocalizer implements Localizer {
    public final Encoder frontLeftWheelEncoder;
    public final Encoder frontRightWheelEncoder;
    public final Encoder backLeftWheelEncoder;
    public final Encoder backRightWheelEncoder;

    private int lastFrontLeftPosition;
    private int lastFrontRightPosition;
    private int lastBackLeftPosition;
    private int lastBackRightPosition;
    private Rotation2d lastHeading;

    private final IMU imu;

    public MecanumLocalizer(HardwareManager hardwareManager) {
        frontLeftWheelEncoder  = new OverflowEncoder(new RawEncoder(hardwareManager.getFrontLeftWheelMotor()));
        frontRightWheelEncoder = new OverflowEncoder(new RawEncoder(hardwareManager.getFrontRightWheelMotor()));
        backLeftWheelEncoder   = new OverflowEncoder(new RawEncoder(hardwareManager.getBackLeftWheelMotor()));
        backRightWheelEncoder  = new OverflowEncoder(new RawEncoder(hardwareManager.getBackRightWheelMotor()));
        imu = hardwareManager.lazyIMU.get();

        // Set Initial measurements
        lastFrontLeftPosition  = frontLeftWheelEncoder.getPositionAndVelocity().position;
        lastFrontRightPosition = frontRightWheelEncoder.getPositionAndVelocity().position;
        lastBackLeftPosition   = backLeftWheelEncoder.getPositionAndVelocity().position;
        lastBackRightPosition  = backRightWheelEncoder.getPositionAndVelocity().position;
        lastHeading = getCurrentIMUHeading();
    }

    @Override
    public Twist2dDual<Time> update() {
        PositionVelocityPair frontLeftPosVel  = frontLeftWheelEncoder.getPositionAndVelocity();
        PositionVelocityPair frontRightPosVel = frontRightWheelEncoder.getPositionAndVelocity();
        PositionVelocityPair backLeftPosVel   = backLeftWheelEncoder.getPositionAndVelocity();
        PositionVelocityPair backRightPosVel  = backRightWheelEncoder.getPositionAndVelocity();

        Rotation2d heading = getCurrentIMUHeading();
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = GlobalConfig.MecanumDriveConfig.kinematics.forward(
            new MecanumKinematics.WheelIncrements<>(
                new DualNum<Time>(new double[] {
                    (frontLeftPosVel.position - lastFrontLeftPosition),
                    frontLeftPosVel.velocity,
                }).times(GlobalConfig.MecanumDriveConfig.inchesPerTick),
                new DualNum<Time>(new double[] {
                    (backLeftPosVel.position - lastBackLeftPosition),
                    backLeftPosVel.velocity,
                }).times(GlobalConfig.MecanumDriveConfig.inchesPerTick),
                new DualNum<Time>(new double[] {
                    (backRightPosVel.position - lastBackRightPosition),
                    backRightPosVel.velocity,
                }).times(GlobalConfig.MecanumDriveConfig.inchesPerTick),
                new DualNum<Time>(new double[] {
                    (frontRightPosVel.position - lastFrontRightPosition),
                    frontRightPosVel.velocity,
                }).times(GlobalConfig.MecanumDriveConfig.inchesPerTick)
            )
        );

        lastFrontLeftPosition  = frontLeftPosVel.position;
        lastFrontRightPosition = frontRightPosVel.position;
        lastBackLeftPosition   = backLeftPosVel.position;
        lastBackRightPosition  = backRightPosVel.position;
        lastHeading = heading;

        return new Twist2dDual<>(
            twist.line,
            DualNum.cons(headingDelta, twist.angle.drop(1))
        );
    }

    /** @return the current IMU heading in {@link Rotation2d} */
    private Rotation2d getCurrentIMUHeading() {
        return Rotation2d.exp(imu
            .getRobotYawPitchRollAngles()
            .getYaw(AngleUnit.RADIANS)
        );
    }
}
