package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.all_purpose.ComputerVision;
import org.firstinspires.ftc.teamcode.all_purpose.HardwareManager;
import org.firstinspires.ftc.teamcode.base.SelfDriving;

@Autonomous(name = "OpenCV Test", group = "Test")
public class OpenCV extends SelfDriving {
    ComputerVision computerVision;

    @Override
    public void runOpMode() {
        hardwareManager = new HardwareManager(hardwareMap);
        computerVision = new ComputerVision(hardwareMap, true);

        waitForStart();
        runAutonomous();
    }

    @Override
    protected void runAutonomous() {
        computerVision.start();
    }
}
