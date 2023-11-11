package org.firstinspires.ftc.teamcode.self_driving;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.base.SelfDriving;

/**
 * Starting from the back (Closest from the boards), facing
 * the boards, this code parks the bot in the back parking.
 */
@Autonomous(name = "Back Parking", group = "Autonomous")
public class BackParking extends SelfDriving {
    @Override
    protected void runAutonomous() {
        move(1.2);
        sleep(250);
        move(-0.1);
    }
}
