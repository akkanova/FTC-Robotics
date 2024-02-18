package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.jetbrains.annotations.NotNull;

import java.util.function.Consumer;

/** Class containing all the required setup and utility code for various hardware bindings. */
public class HardwareManager {
    //-----------------------------------------------------------------------------------
    // Sensors
    //-----------------------------------------------------------------------------------

    /** Inertial measurement unit, that is only loaded when required (lazily loaded) */
    public final LazyImu lazyIMU;
    public final VoltageSensor batteryVoltageSensor;

    //-----------------------------------------------------------------------------------
    // Drive Base
    //-----------------------------------------------------------------------------------

    public final DcMotorEx[] wheelMotors;
    public final DcMotorEx getFrontLeftWheelMotor()  { return wheelMotors[0]; } // TETRIX TorqueNADO 40:1
    public final DcMotorEx getFrontRightWheelMotor() { return wheelMotors[1]; } // TETRIX TorqueNADO 40:1
    public final DcMotorEx getBackLeftWheelMotor()   { return wheelMotors[2]; } // TETRIX TorqueNADO 40:1
    public final DcMotorEx getBackRightWheelMotor()  { return wheelMotors[3]; } // TETRIX TorqueNADO 40:1

    /** Run consumer callback for all elements of the wheel motors array */
    public void doForAllWheels(@NotNull Consumer<DcMotorEx> consumer) {
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

    public HardwareManager(@NotNull HardwareMap hardwareMap) {
        // Lynx Modules (a.k.a Control Hub or Expansion Hub) ----------------------------
        // Ensure that all the lynx modules have the minimum firmware version
        // required by Road Runner.
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        // More Info : https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Sensors ----------------------------------------------------------------------
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        lazyIMU = new LazyImu(hardwareMap, "imu",
            new RevHubOrientationOnRobot(
                // Orientation of the REV Control Hub's Logo relative to the Robot's body
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                // Orientation of the REV Control Hub's USB ports relative to the Robot's body
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            )
        );

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

        // Arm --------------------------------------------------------------------------
        clawServoLeft = hardwareMap.get(ServoImplEx.class, "ClawLeftS");
        clawServoRight = hardwareMap.get(ServoImplEx.class, "ClawRightS");
        clawServoLeft.setDirection(Servo.Direction.REVERSE);
        clawServoRight.setDirection(Servo.Direction.FORWARD);

        armElbowMotor = hardwareMap.get(DcMotorEx.class, "ElbowArmM");
        armElbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Drone Launcher ---------------------------------------------------------------
        //droneLauncherBase = hardwareMap.get(ServoImplEx.class, "LauncherBaseS");
        //droneLauncherHook = hardwareMap.get(ServoImplEx.class, "LauncherHookS");
        //droneLauncherBase.setDirection(Servo.Direction.FORWARD);
        //droneLauncherHook.setDirection(Servo.Direction.REVERSE);

        // Lift -------------------------------------------------------------------------
        liftMotor = hardwareMap.get(DcMotorEx.class, "LiftM");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
