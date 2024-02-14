package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.teamcode.common.hardware.SimpleMecanumDrive;

/**
 * One stop shop for all dynamically configurable
 * variables that exists in our codebase.
 */
public class Configuration {
    /**
     * Physical measurements and Hardware Specified Constants for
     * the Drive base. As required by Road Runner.
     * */
    public static class DriveBaseConstants {
        /** Specifically for a TETRIX TorqueNADO 40:1 */
        public static final float MOTORS_TICKS_PER_REVOLUTION = 960;
        /** Specifically for a TETRIX TorqueNADO 40:1 */
        public static final float MOTORS_MAX_RPM = 150;
        /** Not including the Motor's gearbox (output / input) */
        public static final float GEAR_RATIO = 1;
        /** Specifically for a Mecanum 100mm Wheel (inches) */
        public static final double WHEEL_RADIUS = 1.968504;
        /** Distance between the left and right wheel centers (inches) */
        public static final double TRACK_WIDTH = 15.5;
        /** Distance between two wheel centers of the same side (inches) */
        public static final double TRACK_BASE = 11;

        /**
         * Due to IRL factors you'd only technically be able to achieve ~ 80 to 90% of your
         * theoretically max speed (inches / second)..
         * */
        public static double MAX_VELOCITY = rpmToInchesPerSecond(MOTORS_MAX_RPM) * 0.85;
        /** (inches / second ^ 2) @todo refine later.. */
        public static double MAX_ACCELERATION = MAX_VELOCITY;
        /** (radians / second) */
        public static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / TRACK_WIDTH;
        /** (radians / second ^ 2) */
        public static double MAX_ANGULAR_ACCELERATION = MAX_ACCELERATION / TRACK_WIDTH;

        /**
         * @return velocity (inches / second) converted from the provided RPM of a
         * motor from our drive base.
         * */
        public static double rpmToInchesPerSecond(double rpm) {
            return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
        }

        /**
         * @return distance traveled (inches) from the provided encoder "ticks" of a
         * motor from our drive base.
         * */
        public static double encoderTicksToInches(double ticks) {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTORS_MAX_RPM;
        }

        /**
         * @return distance traveled (meters) from the provided encoder "ticks" of a
         * motor from our drive base.
         */
        public static double encoderTicksToMeters(double ticks) {
            return encoderTicksToInches(ticks) * 0.0254;
        }
    }

    /**
     * The Control Hub takes each motors' encoder values as Input and decides on the
     * appropriate voltage to give them through some "arbitrary calculations" that utilizes
     * these PID Coefficients to maintain a specified target velocity.<br>
     * More Info
     * <a href="https://gm0.org/en/latest/docs/software/concepts/control-loops.html">
     *     Game Manual 0 - PID Controllers
     * </a> and
     * <a href="https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.h2mitzlvr4py">
     *      Motor PIDF Tuning Guide Google Doc
     * </a>.<br>

     *  @todo Actually tune these coefficients using DriveVelocityPIDTuner TeleOp.
     */
    @Config
    public static class BuiltInVelocityPID {
        public static float f = 32767 / (
                DriveBaseConstants.MOTORS_MAX_RPM /
                60 * DriveBaseConstants.MOTORS_TICKS_PER_REVOLUTION);
        public static double p = f * 0.1;
        public static double i = p * 0.1;
        public static double d = 0;
    }

    /**
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    @Config
    public static class FeedForwardPID {
        public static double kStatic = 0;
        public static double kA = 0;
        public static double kV = 1.0 /
                DriveBaseConstants.rpmToInchesPerSecond(
                        DriveBaseConstants.MOTORS_MAX_RPM);
    }

    /** Config specifically for {@link SimpleMecanumDrive} */
    @Config
    public static class MecanumDriveBaseConfig {
        /**
         * Factor that multiplies strafe velocity to compensate for slip; increase it to boost the
         * distance traveled in the strafe direction
         */
        public static double LATERAL_MULTIPLIER = 1;
        public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
        public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
    }
}
