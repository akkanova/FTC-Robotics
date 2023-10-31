package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Main Autonomous", group = "Autonomous")
public class MainAutonomous extends Root {
    private final double MECANUM_WHEEL_CIRCUMFERENCE = Math.PI * 0.098; // (m) https://www.pitsco.com/TETRIX-MAX-Mecanum-Wheels
    private final int TETRIX_MOTOR_CPR = 1440; // https://asset.pitsco.com/sharedimages/resources/torquenado_dcmotorspecs.pdf

    private ElapsedTime totalRuntime;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        setupPreciseDcMotors();
        sendInitialTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        totalRuntime = new ElapsedTime();

        // Autonomous code here..
        // Since it's executed sequentially, write it here.

        revolve(frontLeftWheel,  1);
        revolve(frontRightWheel, 1);
        revolve(backLeftWheel,   1);
        revolve(backRightWheel,  1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status","Total Runtime %.3f s", totalRuntime.seconds());
        appendMotorDebugTelemetry();
        telemetry.update();
    }

    // Run the specified motor until it moved this amount of distance
    private void revolve(DcMotor motor, double metersDistance) {
        double currentRevolutions = motor.getCurrentPosition() / TETRIX_MOTOR_CPR;
        double circumference = Math.PI * MECANUM_WHEEL_CIRCUMFERENCE;

        double requiredRevolutions = metersDistance / circumference;
        double totalRevolutions = currentRevolutions + requiredRevolutions;

        motor.setPower(0.5);
        motor.setTargetPosition((int) totalRevolutions);
    }
}
