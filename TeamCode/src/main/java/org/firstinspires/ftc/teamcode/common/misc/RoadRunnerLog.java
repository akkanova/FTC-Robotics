package org.firstinspires.ftc.teamcode.common.misc;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;

/**
 * Class for containing all the special formatters Road Runner utilizes
 * for its verbose logging.
 * */
public final class RoadRunnerLog {
    /** Logger specifically for {@link MecanumDrive} */
    public static class MecanumDriveLogger {
        public final DownsampledWriter estimatedPoseWriter;
        public final DownsampledWriter targetPoseWriter;
        public final DownsampledWriter driveCommandWriter;
        public final DownsampledWriter mecanumCommandWriter;

        public MecanumDriveLogger() {
            estimatedPoseWriter  = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
            targetPoseWriter     = new DownsampledWriter("TARGET_POSE", 50_000_000);
            driveCommandWriter   = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
            mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);
        }
    }

    /**
     * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/messages/MecanumLocalizerInputsMessage.java">
     *      Copied from Road Runner quickstart  - MecanumLocalizerInputsMessage.java
     * </a>
     */
    public static final class MecanumLocalizerInputsLogMessage {
        // A: I Curse Java 8 for not having Records yet!

        public long timestamp;
        public PositionVelocityPair frontLeft;
        public PositionVelocityPair frontRight;
        public PositionVelocityPair backLeft;
        public PositionVelocityPair backRight;

        public double yaw;
        public double pitch;
        public double roll;

        public MecanumLocalizerInputsLogMessage(
            PositionVelocityPair frontLeft,
            PositionVelocityPair frontRight,
            PositionVelocityPair backLeft,
            PositionVelocityPair backRight,
            YawPitchRollAngles angles
        ) {
            this.timestamp  = System.nanoTime();
            this.frontLeft  = frontLeft;
            this.frontRight = frontRight;
            this.backLeft   = backLeft;
            this.backRight  = backRight;
            this.yaw   = angles.getYaw(AngleUnit.RADIANS);
            this.pitch = angles.getPitch(AngleUnit.RADIANS);
            this.roll  = angles.getRoll(AngleUnit.RADIANS);
        }
    }

    /**
     * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/messages/PoseMessage.java">
     *      Copied from Road Runner quickstart  -  PoseMessage.java
     * </a>
     * */
    public static final class PoseLogMessage {

        public long timestamp;
        public double x;
        public double y;
        public double heading;

        public PoseLogMessage(Pose2d pose) {
            this.timestamp = System.nanoTime();
            this.heading = pose.heading.toDouble();
            this.x = pose.position.x;
            this.y = pose.position.y;
        }
    }

    /**
     * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/messages/DriveCommandMessage.java">
     *      Copied from Road Runner quickstart  -  DriveCommandMessage.java
     * </a>
     */
    public static final class DriveCommandLogMessage {
        public long timestamp;
        public double forwardVelocity;
        public double lateralVelocity;
        public double angularVelocity;

        public double forwardAcceleration;
        public double lateralAcceleration;
        public double angularAcceleration;

        public DriveCommandLogMessage(PoseVelocity2dDual<Time> poseVelocity) {
            this.timestamp = System.nanoTime();
            this.forwardVelocity = poseVelocity.linearVel.x.get(0);
            this.lateralVelocity = poseVelocity.linearVel.y.get(0);
            this.angularVelocity = poseVelocity.angVel.get(0);
            this.forwardAcceleration = poseVelocity.linearVel.x.get(1);
            this.lateralAcceleration = poseVelocity.linearVel.y.get(1);
            this.angularAcceleration = poseVelocity.angVel.get(1);
        }
    }

    /**
     * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/messages/MecanumCommandMessage.java">
     *      Copied from Road Runner quickstart  -  MecanumCommandMessage.java
     * </a>
     */
    public static final class MecanumCommandLogMessage {
        public long timestamp;
        public double voltage;
        public double frontLeftPower;
        public double frontRightPower;
        public double backLeftPower;
        public double backRightPower;

        public MecanumCommandLogMessage(
            double voltage,
            double frontLeftPower,
            double frontRightPower,
            double backLeftPower,
            double backRightPower
        ) {
            this.timestamp = System.nanoTime();
            this.voltage = voltage;

            this.frontLeftPower  = frontLeftPower;
            this.frontRightPower = frontRightPower;
            this.backLeftPower   = backLeftPower;
            this.backRightPower  = backRightPower;
        }
    }

    /**
     * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/messages/ThreeDeadWheelInputsMessage.java">
     *      Copied from Road Runner quickstart  -  ThreeDeadWheelInputsMessage.java
     * </a>
     * */
    public static final class ThreeDeadWheelInputsLogMessage {
        public long timestamp;
        public PositionVelocityPair parallelLeft;
        public PositionVelocityPair parallelRight;
        public PositionVelocityPair perpendicular;

        public ThreeDeadWheelInputsLogMessage(
            PositionVelocityPair parallelLeft,
            PositionVelocityPair parallelRight,
            PositionVelocityPair perpendicular
        ) {
            this.timestamp = System.nanoTime();
            this.parallelLeft  = parallelLeft;
            this.parallelRight = parallelRight;
            this.perpendicular = perpendicular;
        }
    }
}
