package org.firstinspires.ftc.teamcode.self_driving;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.base.SelfDriving;

/**
 * Starting from the right side (viewed from audience), then
 * parks in the back through the swivel door.
 *
 * Work in progress.. Current only parks in front of the gate,
 * and nothing else. Which gains no points.
 */
@Autonomous(name = "Right Start Parking", group = "Autonomous")
public class RightStartParking extends SelfDriving {
    @Override
    protected void runAutonomous() {
        move(1.4);
        rotate(90);
        move(2.6);
        rotate(45);
        //openClaw();
    }
}
