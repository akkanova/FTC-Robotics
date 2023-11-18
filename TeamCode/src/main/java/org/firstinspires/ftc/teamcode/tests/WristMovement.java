package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.base.SelfDriving;

/*[Script for autonomous arm movement]

 */
@Autonomous(name = "Wrist Movement Test", group = "Test")
public class WristMovement extends SelfDriving {
    @Override
    protected void runAutonomous() {
        keepTurningTillSeconds(1000);
    }
}
