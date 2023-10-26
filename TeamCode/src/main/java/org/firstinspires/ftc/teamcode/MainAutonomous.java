package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Main Autonomous", group = "Autonomous")
public class MainAutonomous extends Root {
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status","Total Runtime %.3f s", totalRuntime.seconds());

        telemetry.addData("FL Raw Current Position", frontLeftM.getCurrentPosition());
        telemetry.addData("FR Raw Current Position", frontRightM.getCurrentPosition());
        telemetry.addData("BL Raw Current Position", backLeftM.getCurrentPosition());
        telemetry.addData("BR Raw Current Position", backRightM.getCurrentPosition());

        telemetry.addData("FL Normalized Angle", normalizedAngle(frontLeftM.getCurrentPosition()));
        telemetry.addData("FR Normalized Angle", normalizedAngle(frontRightM.getCurrentPosition()));
        telemetry.addData("BL Normalized Angle", normalizedAngle(backLeftM.getCurrentPosition()));
        telemetry.addData("BR Normalized Angle", normalizedAngle(backRightM.getCurrentPosition()));

        sendMotorDebugTelemetry();
    }

    public double normalizedAngle(int position) {
        double revolutions = position/360;
        return (revolutions * 360) % 360;
    }
}
