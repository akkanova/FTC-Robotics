package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.SelfDriving;


@Autonomous(name = "Accuracy Test", group = "Test")
public class Accuracy extends SelfDriving {
    @Override
    protected void runAutonomous() {
        move(1);
    }
}
