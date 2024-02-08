package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.common.hardware.LynxModuleUtil;

import java.util.function.Consumer;

/**
 * Configured for the Drivetrain and Components used on
 * our CenterStage 2023-2024 participation.<br>

 * When copying and modifying for future use in the next
 * competitions, don't copy everything verbatim unless
 * the the hardware are also the same.
 */
public class HardwareManager {
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
    public static class DriveConstants {
        // Orientation of the Control Hub Parts relative to the Robot's Body...
        public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    }


    //-----------------------------------------------------------------------------------
    // Wheels (Mecanum Wheels & Configuration)
    //-----------------------------------------------------------------------------------

    public final DcMotorEx[] wheels;
    public final DcMotorEx getFrontLeftWheel()  { return wheels[0]; } // TETRIX TorqueNADO 40:1
    public final DcMotorEx getFrontRightWheel() { return wheels[1]; } // TETRIX TorqueNADO 40:1
    public final DcMotorEx getBackLeftWheel()   { return wheels[2]; } // TETRIX TorqueNADO 40:1
    public final DcMotorEx getBackRightWheel()  { return wheels[3]; } // TETRIX TorqueNADO 40:1

    /** Run consumer callback for all elements of the wheels array */
    public void doForAllWheels(Consumer<DcMotorEx> consumer) {
        for (DcMotorEx wheel : wheels) {
            consumer.accept(wheel);
        }
    }


    //-----------------------------------------------------------------------------------
    // Arm Motors
    //-----------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------
    // Arm
    //-----------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------
    // Sensors
    //-----------------------------------------------------------------------------------

    /** 9-axis Orientation Sensor */
    public final IMU imu;
    public final VoltageSensor batteryVoltageSensor;

    public HardwareManager(HardwareMap hardwareMap) {
        // A Lynx Module = Control Hub, or Expansion Hub...
        // Ensure that all the lynx modules (Control Hub, Expansion Hub 1, Expansion Hub 2, etc ...)
        // have the minimum firmware version required by Road Runner.
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // More Info : https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wheels
        wheels = new DcMotorEx[] {
                hardwareMap.get(DcMotorEx.class, "FrontLeftM"),
                hardwareMap.get(DcMotorEx.class, "FrontRightM"),
                hardwareMap.get(DcMotorEx.class, "BackLeftM"),
                hardwareMap.get(DcMotorEx.class, "BackRightM")
        };

        // A: I don't understand why I should allocate them into an intermediate
        // variable before putting them into the wheels array...
        getFrontLeftWheel().setDirection(DcMotorSimple.Direction.REVERSE);
        getFrontRightWheel().setDirection(DcMotorSimple.Direction.FORWARD);
        getBackLeftWheel().setDirection(DcMotorSimple.Direction.REVERSE);
        getBackRightWheel().setDirection(DcMotorSimple.Direction.FORWARD);

        // More Info : https://gm0.org/en/latest/docs/software/adv-control-system/sdk-motors.html
        doForAllWheels((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        doForAllWheels((motor) -> {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1);
            motor.setMotorType(motorConfigurationType);
        });

        // Sensors
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        DriveConstants.LOGO_FACING_DIRECTION,
                        DriveConstants.USB_FACING_DIRECTION
                )
        ));

        imu.resetYaw();
    }
}
