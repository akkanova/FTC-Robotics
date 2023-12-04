package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.base.HumanOperated;

/**
 * Print out every information the software has access to. Big Info dump..
 */
@TeleOp(name="Hardware Debug", group="Test")
public class HardwareDebug extends HumanOperated {
    @Override
    protected void processUserInput() {} // .. Required

    @Override
    public void start() {
        hardwareManager.imu.resetYaw();
        hardwareManager.resetWheelEncoders();
    }

    @Override
    public void loop() {
        // Operator Input
        telemetry
                .addData("GamePad1", gamepad1.id < 0 ? "Not Plugged In." : gamepad1.toString())
                .addData("GamePad2", gamepad2.id < 0 ? "Not Plugged In." : gamepad2.toString());

        YawPitchRollAngles orientation = hardwareManager.imu.getRobotYawPitchRollAngles();

        // Sensors
        telemetry
                .addData("Yaw",     "%.3f°", orientation.getYaw(AngleUnit.DEGREES))
                .addData("Roll",    "%.3f°", orientation.getRoll(AngleUnit.DEGREES))
                .addData("Pitch",   "%.3f°", orientation.getPitch(AngleUnit.DEGREES))
                .addData("Voltage", "%.3f Volts", hardwareManager.voltageSensor.getVoltage());

        // Hardware Components
        telemetry
                .addData("Front-Left Wheel",  hardwareManager.getFrontLeftWheel().getCurrentPosition())
                .addData("Front-Right Wheel", hardwareManager.getFrontRightWheel().getCurrentPosition())
                .addData("Back-Left Wheel",   hardwareManager.getBackLeftWheel().getCurrentPosition())
                .addData("Back-Right Wheel",  hardwareManager.getBackRightWheel().getCurrentPosition())
                .addData("Bottom Arm Motor",  hardwareManager.bottomArmMotor.getCurrentPosition())
                .addData("Lift Motor",        hardwareManager.liftMotor.getCurrentPosition());

        telemetry.update();
    }
}
