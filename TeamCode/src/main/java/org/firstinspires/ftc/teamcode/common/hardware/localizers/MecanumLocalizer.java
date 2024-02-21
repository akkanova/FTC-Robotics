package org.firstinspires.ftc.teamcode.common.hardware.localizers;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.misc.RoadRunnerLog;

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
    private boolean initialized;

    private final IMU imu;

    public MecanumLocalizer(HardwareManager hardwareManager) {
        frontLeftWheelEncoder  = new OverflowEncoder(new RawEncoder(hardwareManager.getFrontLeftWheelMotor()));
        frontRightWheelEncoder = new OverflowEncoder(new RawEncoder(hardwareManager.getFrontRightWheelMotor()));
        backLeftWheelEncoder   = new OverflowEncoder(new RawEncoder(hardwareManager.getBackLeftWheelMotor()));
        backRightWheelEncoder  = new OverflowEncoder(new RawEncoder(hardwareManager.getBackRightWheelMotor()));
        imu = hardwareManager.lazyIMU.get();

        frontLeftWheelEncoder.setDirection(hardwareManager.getFrontLeftWheelMotor().getDirection());
        frontRightWheelEncoder.setDirection(hardwareManager.getFrontRightWheelMotor().getDirection());
        backLeftWheelEncoder.setDirection(hardwareManager.getBackLeftWheelMotor().getDirection());
        backRightWheelEncoder.setDirection(hardwareManager.getBackRightWheelMotor().getDirection());
    }

    @Override
    public Twist2dDual<Time> update() {
        PositionVelocityPair frontLeftPosVel  = frontLeftWheelEncoder.getPositionAndVelocity();
        PositionVelocityPair frontRightPosVel = frontRightWheelEncoder.getPositionAndVelocity();
        PositionVelocityPair backLeftPosVel   = backLeftWheelEncoder.getPositionAndVelocity();
        PositionVelocityPair backRightPosVel  = backRightWheelEncoder.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        FlightRecorder.write(
            "MECANUM_LOCALIZER_INPUTS",
            new RoadRunnerLog.MecanumLocalizerInputsLogMessage(
                frontLeftPosVel, frontRightPosVel,
                backLeftPosVel, backRightPosVel, angles
            )
        );

        // Only read the initial position when Op mode has started
        // Which will call this.. through "Actions"..
        if (!initialized) {
            initialized = true;

            lastFrontLeftPosition  = frontLeftWheelEncoder.getPositionAndVelocity().position;
            lastFrontRightPosition = frontRightWheelEncoder.getPositionAndVelocity().position;
            lastBackLeftPosition   = backLeftWheelEncoder.getPositionAndVelocity().position;
            lastBackRightPosition  = backRightWheelEncoder.getPositionAndVelocity().position;
            lastHeading = heading;

            return new Twist2dDual<>(
                Vector2dDual.constant(new Vector2d(0, 0), 2),
                DualNum.constant(0, 2)
            );
        }

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
}
