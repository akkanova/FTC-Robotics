package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.hardware.PIDFController;
import org.jetbrains.annotations.NotNull;

import java.util.function.Consumer;

/** Class containing all the required setup and utility code for various hardware bindings. */
public class HardwareManager {
    //-----------------------------------------------------------------------------------
    // Sensors
    //-----------------------------------------------------------------------------------

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

    public final ServoImplEx leftClawServo;  // Studica Multi-Mode Smart Servo
    public final ServoImplEx rightClawServo; // Studica Multi-Mode Smart Servo
    public final CRServoImplEx armExtensionServo; // Vex 2-Wire Motor
    public final DcMotorEx elbowMotor; // TETRIX TorqueNADO 40:1

    private PIDFController elbowPIDFController;

    /**
     * Uses a dedicated lazy loaded controller, to determine the required
     * amount of power to hold it accurately to that specific degree.
     * @param targetAngle the target angle of the arm from it's stationary base.
     *  */
    public double getRequiredElbowPower(double targetAngle) {
        if (elbowPIDFController == null)
            elbowPIDFController = new PIDFController(
                    GlobalConfig.ElbowMotorConfig.P,
                    GlobalConfig.ElbowMotorConfig.I,
                    GlobalConfig.ElbowMotorConfig.D
            );

        int currentMotorTicks = elbowMotor.getCurrentPosition();
        double targetTicks = (targetAngle - GlobalConfig.ElbowMotorConfig.initialAngle) *
                GlobalConfig.ElbowMotorConfig.TICK_PER_ANGLE;

        // To Compensate for Gravity
        double ff  = Math.cos(Math.toRadians(targetAngle)) * GlobalConfig.ElbowMotorConfig.F;
        double pid = elbowPIDFController.calculate(currentMotorTicks, targetTicks);
        return pid + ff;
    }

    public void holdElbowAt(double targetAngle) {
        if (targetAngle < GlobalConfig.ElbowMotorConfig.THRESHOLD_ANGLE) {
            elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            elbowMotor.setPower(0);
        }

        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setPower(getRequiredElbowPower(targetAngle));
    }

    public double getCurrentArmAngle() {
        return elbowMotor.getCurrentPosition() * GlobalConfig.ElbowMotorConfig.ANGLE_PER_TICK +
                GlobalConfig.ElbowMotorConfig.initialAngle;
    }

    //-----------------------------------------------------------------------------------
    // Drone Launcher
    //-----------------------------------------------------------------------------------

     public final ServoImplEx droneLauncherHook;  // Studica Multi-Mode Smart Servo

    //-----------------------------------------------------------------------------------
    // Lift
    //-----------------------------------------------------------------------------------

    public final DcMotorEx liftMotor; // TETRIX TorqueNADO 40:1

    public HardwareManager(@NotNull HardwareMap hardwareMap) {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Wheels -----------------------------------------------------------------------
        wheelMotors = new DcMotorEx[] {
            hardwareMap.get(DcMotorEx.class, GlobalConfig.HardwareBindingNames.frontLeftWheelMotor),
            hardwareMap.get(DcMotorEx.class, GlobalConfig.HardwareBindingNames.frontRightWheelMotor),
            hardwareMap.get(DcMotorEx.class, GlobalConfig.HardwareBindingNames.backLeftWheelMotor),
            hardwareMap.get(DcMotorEx.class, GlobalConfig.HardwareBindingNames.backRightWheelMotor)
        };

        // A: I don't understand why I should allocate them into an intermediate
        // variable before putting them into the wheels array...
        getFrontLeftWheelMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        getFrontRightWheelMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        getBackLeftWheelMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        getBackRightWheelMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        // More Info : https://gm0.org/en/latest/docs/software/adv-control-system/sdk-motors.html
        doForAllWheels((motor) -> {
            // @todo Remove if using the MecanumLocalizer..
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        // Arm --------------------------------------------------------------------------
        leftClawServo = hardwareMap.get(ServoImplEx.class, GlobalConfig.HardwareBindingNames.leftClawServo);
        rightClawServo = hardwareMap.get(ServoImplEx.class, GlobalConfig.HardwareBindingNames.rightClawServo);
        leftClawServo.setDirection(Servo.Direction.REVERSE);
        rightClawServo.setDirection(Servo.Direction.FORWARD);

        armExtensionServo = hardwareMap.get(CRServoImplEx.class, GlobalConfig.HardwareBindingNames.clawExtenderServo);
        armExtensionServo.setDirection(DcMotor.Direction.FORWARD);

        elbowMotor = hardwareMap.get(DcMotorEx.class, GlobalConfig.HardwareBindingNames.elbowMotor);
        elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drone Launcher ---------------------------------------------------------------
        droneLauncherHook = hardwareMap.get(ServoImplEx.class,
            GlobalConfig.HardwareBindingNames.droneLauncherHookServo);
        droneLauncherHook.setDirection(Servo.Direction.FORWARD);
        droneLauncherHook.setPosition(1);

        // Lift -------------------------------------------------------------------------
        liftMotor = hardwareMap.get(DcMotorEx.class, GlobalConfig.HardwareBindingNames.liftMotor);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
