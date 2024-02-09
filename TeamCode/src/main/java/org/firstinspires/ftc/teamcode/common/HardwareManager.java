package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.common.hardware.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.common.hardware.SimpleMecanumDrive;

import java.util.function.Consumer;

/**
 * Configured for the Drivetrain and Components used on
 * our CenterStage 2023-2024 participation.<br>

 * When copying and modifying for future use in the next
 * competitions, don't copy everything verbatim unless
 * the the hardware are also the same.

 * interior width
 */
public class HardwareManager {
    //-----------------------------------------------------------------------------------
    // Sensors
    //-----------------------------------------------------------------------------------

    /** 9-axis Orientation Sensor */
    public final IMU imu;
    public final VoltageSensor batteryVoltageSensor;

    //-----------------------------------------------------------------------------------
    // Drive Base
    //-----------------------------------------------------------------------------------

    /**
     * Configured using :<br>
     * <a href="https://learnroadrunner.com/drive-constants.html#drive-constants">
     *     "Learn Road Runner" Website - Drive Constants Automatic Tuner
     * </a>
     * and
     * <a href="http://192.168.43.1:8080/dash">
     *     Our Control Hub's FTC Dashboard
     * </a>
     */
    @Config
    public static class DriveBaseConstants {
        //-------------------------------------------------------------------------------
        // Physical Constants
        //-------------------------------------------------------------------------------

        /** Specifications for a TETRIX TorqueNADO 40:1 */
        public static final float TICKS_PER_REVOLUTION = 960;
        /** Specifications for a TETRIX TorqueNADO 40:1 */
        public static final float MAX_RPM = 150;
        /** Not including the Motor's gearbox (output / input) */
        public static final float GEAR_RATIO = 1;
        /** Studica Mecanum 100mm Wheel (inches) */
        public static final double WHEEL_RADIUS = 1.968504;
        /** Distance between the left and right wheel centers (inches) */
        public static final double TRACK_WIDTH = 15.5;
        /** Distance between two wheel centers of the same side (inches) */
        public static final double TRACK_BASE = 11;

        //-------------------------------------------------------------------------------
        // Calculated Constants
        //-------------------------------------------------------------------------------

        /**
         * Due to IRL factors you'd technically be able to achieve ~ 80 to 90% of your
         * theoretically speed (inches / second)..
         * */
        public static double MAX_VELOCITY = rpmToInchesPerSecond(MAX_RPM) * 0.85;
        /** (inches / second ^ 2) @todo refine later.. */
        public static double MAX_ACCELERATION = MAX_VELOCITY;
        /** (radians / second) */
        public static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / TRACK_WIDTH;
        /** (radians / second ^ 2) */
        public static double MAX_ANGULAR_ACCELERATION = MAX_ACCELERATION / TRACK_WIDTH;

        /**
         * @return velocity (inches / second) converted from the provided RPM of a
         * motor of our drive base.
         * */
        public static double rpmToInchesPerSecond(double rpm) {
            return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
        }

        /**
         * @return distance traveled (inches) from the provided encoder "ticks" of a
         * motor of our drive base.
         * */
        public static double encoderTicksToInches(double ticks) {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REVOLUTION;
        }

        //-------------------------------------------------------------------------------
        // Wheel Motors Velocity PIDF
        //-------------------------------------------------------------------------------

        /** The f value in Motor Velocity PIDF Coefficients */
        private static final double f = 32767 / (MAX_RPM / 60 * TICKS_PER_REVOLUTION);
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

         * @todo Actually tune these coefficients using DriveVelocityPIDTuner TeleOp.
         */
        public static final PIDFCoefficients MOTOR_VELOCITY_PIDF =
                new PIDFCoefficients(f * 0.1, f * 0.01, 0, f);

        //-------------------------------------------------------------------------------
        // Feed Forward PID Parameters
        //-------------------------------------------------------------------------------
        // These are the feedforward parameters used to model the drive motor behavior. If you are using
        // the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
        // motor encoders or have elected not to use them for velocity control, these values should be
        // empirically tuned.                      -- Road Runner

        public static double kV = 1.0 / rpmToInchesPerSecond(MAX_RPM);
        public static double kA = 0;
        public static double kStatic = 0;

        //-------------------------------------------------------------------------------
        // Compensations
        //-------------------------------------------------------------------------------

        /**
         * Factor that multiplies strafe velocity to compensate for slip; increase it to boost the
         * distance traveled in the strafe direction
         */
        public static double LATERAL_MULTIPLIER = 1;
    }

    public final SimpleMecanumDrive driveBase;

    public final DcMotorEx[] wheelMotors;
    public final DcMotorEx getFrontLeftWheelMotor()  { return wheelMotors[0]; } // TETRIX TorqueNADO 40:1
    public final DcMotorEx getFrontRightWheelMotor() { return wheelMotors[1]; } // TETRIX TorqueNADO 40:1
    public final DcMotorEx getBackLeftWheelMotor()   { return wheelMotors[2]; } // TETRIX TorqueNADO 40:1
    public final DcMotorEx getBackRightWheelMotor()  { return wheelMotors[3]; } // TETRIX TorqueNADO 40:1

    /** Run consumer callback for all elements of the wheels array */
    public void doForAllWheels(Consumer<DcMotorEx> consumer) {
        for (DcMotorEx wheel : wheelMotors) {
            consumer.accept(wheel);
        }
    }

    //-----------------------------------------------------------------------------------
    // Arm
    //-----------------------------------------------------------------------------------

    public final ServoImplEx clawServoLeft;  // Studica Multi-Mode Smart Servo
    public final ServoImplEx clawServoRight; // Studica Multi-Mode Smart Servo
    public final DcMotorEx armElbowMotor;    // TETRIX TorqueNADO 40:1

    //-----------------------------------------------------------------------------------
    // Drone Launcher
    //-----------------------------------------------------------------------------------

    // public final ServoImplEx droneLauncherBase;
    // public final ServoImplEx droneLauncherHook;

    //-----------------------------------------------------------------------------------
    // Lift
    //-----------------------------------------------------------------------------------

    public final DcMotorEx liftMotor; // TETRIX TorqueNADO 40:1

    public HardwareManager(HardwareMap hardwareMap) {
        // Lynx Modules (a.k.a Control Hub or Expansion Hub) ----------------------------
        // Ensure that all the lynx modules have the minimum firmware version
        // required by Road Runner.
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // More Info : https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Sensors ----------------------------------------------------------------------
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        // Orientation of the REV Control Hub's Logo relative to the Robot's body
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        // Orientation of the REV Control Hub's USB ports relative to the Robot's body
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));

        imu.resetYaw();

        // Wheels -----------------------------------------------------------------------
        wheelMotors = new DcMotorEx[] {
                hardwareMap.get(DcMotorEx.class, "FrontLeftM"),
                hardwareMap.get(DcMotorEx.class, "FrontRightM"),
                hardwareMap.get(DcMotorEx.class, "BackLeftM"),
                hardwareMap.get(DcMotorEx.class, "BackRightM")
        };

        // A: I don't understand why I should allocate them into an intermediate
        // variable before putting them into the wheels array...
        getFrontLeftWheelMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        getFrontRightWheelMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        getBackLeftWheelMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        getBackRightWheelMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        // More Info : https://gm0.org/en/latest/docs/software/adv-control-system/sdk-motors.html
        doForAllWheels((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        doForAllWheels((motor) -> {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1);
            motor.setMotorType(motorConfigurationType);
        });

        // Notice: ONLY COMMENT OUT IF NOT USING ENCODERS AND HAS COMPLETELY SWITCHED TO
        // DEAD WHEELS ONLY..
        doForAllWheels((motor) -> {
            // Use the Control Hub's Built In Velocity Control PID with PID Coefficients we have
            // fine tuned... look at the comment in `DriveConstants.MOTOR_VELOCITY_PID` for more info.
            motor.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(
                            DriveBaseConstants.MOTOR_VELOCITY_PIDF.p,
                            DriveBaseConstants.MOTOR_VELOCITY_PIDF.i,
                            DriveBaseConstants.MOTOR_VELOCITY_PIDF.d,
                            // Scale based on current battery percentage ..
                            DriveBaseConstants.MOTOR_VELOCITY_PIDF.f * 12 / batteryVoltageSensor.getVoltage()
                    )
            );
        });

        driveBase = new SimpleMecanumDrive(this);

        // Arm --------------------------------------------------------------------------
        clawServoLeft = hardwareMap.get(ServoImplEx.class, "ClawLeftS");
        clawServoRight = hardwareMap.get(ServoImplEx.class, "ClawRightS");
        clawServoLeft.setDirection(Servo.Direction.REVERSE);
        clawServoRight.setDirection(Servo.Direction.FORWARD);

        // @todo Setup PIDF Coefficients for motor
        armElbowMotor = hardwareMap.get(DcMotorEx.class, "ElbowArmM");
        armElbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Drone Launcher ---------------------------------------------------------------
        //droneLauncherBase = hardwareMap.get(ServoImplEx.class, "LauncherBaseS");
        //droneLauncherHook = hardwareMap.get(ServoImplEx.class, "LauncherHookS");
        //droneLauncherBase.setDirection(Servo.Direction.FORWARD);
        //droneLauncherHook.setDirection(Servo.Direction.REVERSE);

        // Lift -------------------------------------------------------------------------
        // @todo Setup PID Coefficients for motor
        liftMotor = hardwareMap.get(DcMotorEx.class, "LiftM");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
