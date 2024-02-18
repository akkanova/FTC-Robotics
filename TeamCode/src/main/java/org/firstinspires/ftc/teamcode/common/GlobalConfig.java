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

import java.util.Arrays;

/**
 * One stop shop for all dynamically configurable
 * variables that exists in our codebase.
 */
public final class GlobalConfig {
    /**
     * Configuration specifically for a {@link MecanumDrive}.
     * Fine tune variables located here using these instructions
     * <a href="https://rr.brott.dev/docs/v1-0/tuning">
     *      Road Runner v1.0.x Tuning Documentation
     * </a>
     * */
    @Config
    public static final class MecanumDriveConfig {
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
}
