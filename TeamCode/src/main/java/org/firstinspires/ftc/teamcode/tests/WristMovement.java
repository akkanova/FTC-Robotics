package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.SelfDriving;

/**
 * Testing for Wrist
 */
@Autonomous(name = "Wrist Movement Test", group = "Test")
public class WristMovement extends SelfDriving {
    @Override
    protected void runAutonomous() {
        moveElbowMotors(45);
    }
}
