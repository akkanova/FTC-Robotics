package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.ThreeWheelLocalizer;

import java.util.Arrays;

/**
 * One stop shop for all <b>dynamically</b> configurable
 * variables that exists in our codebase.
 */
public final class GlobalConfig {
    /** Name of that component in the Control Hub's Hardware Configuration */
    public static final class HardwareBindingNames {
        public static final String imu = "imu";

        public static final String frontLeftWheelMotor  = "FrontLeftM";
        public static final String frontRightWheelMotor = "FrontRightM";
        public static final String backLeftWheelMotor   = "BackLeftM";
        public static final String backRightWheelMotor  = "BackRightM";

        public static final String deadWheelLeftEncoder = "DeadWheelLeftE";
        public static final String deadWheelRightEncoder = "DeadWheelRightE";
        public static final String deadWheelPerpendicularEncoder = "DeadWheelPerpendicularE";

        public static final String leftClawServo = "ClawLeftS";
        public static final String rightClawServo = "ClawRightS";
        public static final String elbowMotor = "ElbowM";

        public static final String droneLauncherServo = "";
        public static final String droneLauncherHookServo = "";

        public static final String liftMotor = "LiftM";
    }

    /**
     * Configuration specifically for {@link MecanumDrive}.
     * Fine tune variables located here using these instructions
     * <a href="https://rr.brott.dev/docs/v1-0/tuning">
     *      Road Runner v1.0.x Tuning Documentation
     * </a>
     * */
    @Config
    public static final class MecanumDriveConfig {
        /** How many poses it will store in it's history before it starts pruning old ones..*/
        public static int maxPoseHistory = 100;
        /** Only used by FTC Dashboard to represent the size of the robot */
        public static int robotRadius = 9;

        /** Determined through
         * <a href="https://rr.brott.dev/docs/v1-0/tuning#forwardpushtest">
         *     ForwardPushTest
         * </a>
         * or manually calculated.. */
        public static double inchesPerTick = 1;
        /** Determined through
         * <a href="https://rr.brott.dev/docs/v1-0/tuning#lateralpushtest-mecanum--drive-encoders-only">
         *     LateralPushTest
         * </a>. */
        public static double lateralInchesPerTick = 1;
        /** Determined through
         * <a href="https://rr.brott.dev/docs/v1-0/tuning#angularramplogger">
         *  AngularRampLogger
         * </a>. */
        public static double trackWidthTicks = 0;

        //-------------------------------------------------------------------------------
        // Feed-Forward Parameters
        //-------------------------------------------------------------------------------
        // Tuned using the Manual feed-forward tuner
        // (https://rr.brott.dev/docs/v1-0/tuning#manualfeedforwardtuner)

        /** Feed forward parameter, In ticks unit */
        public static double kS = 0;
        /** Feed forward parameter, In ticks unit */
        public static double kV = 0;
        /** Feed forward parameter, In ticks unit */
        public static double kA = 0;

        //-------------------------------------------------------------------------------
        // Path Profile Parameters
        //-------------------------------------------------------------------------------

        /** In inches / second */
        public static double maxWheelVelocity = 50;
        /** In inches / second^2 */
        public static double minProfileAcceleration = -30;
        /** In inches / second^2 */
        public static double maxProfileAcceleration = 50;

        //-------------------------------------------------------------------------------
        // Turn Profile Parameters
        //-------------------------------------------------------------------------------

        /** In radians / second */
        public static double maxAngularVelocity = Math.PI;
        /** In radians / second^2 */
        public static double maxAngularAcceleration = Math.PI;

        //-------------------------------------------------------------------------------
        // Path Controller Gains
        //-------------------------------------------------------------------------------
        // For fine tuning the path follower

        public static double axialGain = 0;
        public static double lateralGain = 0;
        public static double headingGain = 0;

        public static double axialVelocityGain = 0;
        public static double lateralVelocityGain = 0;
        public static double headingVelocityGain = 0;

        //-------------------------------------------------------------------------------
        // Calculated Variables
        //-------------------------------------------------------------------------------

        public static final MecanumKinematics kinematics =
            new MecanumKinematics(
                inchesPerTick * trackWidthTicks,
                inchesPerTick / lateralInchesPerTick
            );

        public static final TurnConstraints defaultTurnConstraints =
            new TurnConstraints(
                maxAngularVelocity,
                -maxAngularAcceleration,
                maxAngularAcceleration
            );

        public static final VelConstraint defaultVelocityConstraint =
            new MinVelConstraint(Arrays.asList(
                // A: Java what is this?
                kinematics.new WheelVelConstraint(maxWheelVelocity),
                new AngularVelConstraint(maxAngularVelocity)
            ));

        public static final AccelConstraint defaultAccelerationConstraint =
            new ProfileAccelConstraint(
                minProfileAcceleration,
                maxProfileAcceleration
            );
    }

    /** Configuration specifically for {@link ThreeWheelLocalizer}*/
    @Config
    public static final class ThreeWheelLocalizerConfig {
        public static double inchesPerTick = 1;

        /** y position of the left parallel encoder (in tick units) */
        public static double leftParallelYTicks = 0.0;
        /** y position of the right parallel encoder (in tick units) */
        public static double rightParallelYTicks = 1.0;
        /** x position of the perpendicular encoder (in tick units) */
        public static double perpendicularXTicks = 0.0;
    }

    /** Determines whether Human-Operated TeleOps should not be registered */
    public static final boolean DISABLE_OPERATED_OP_MODES = false;

    /** Determines whether Road Runner tuning TeleOPs should not be registered */
    public static final boolean DISABLE_ROAD_RUNNER_TUNING = false;

    /** Determines whether Debug TeleOps should not be registered */
    public static final boolean DISABLE_DEBUG_OP_MODES = false;
}
