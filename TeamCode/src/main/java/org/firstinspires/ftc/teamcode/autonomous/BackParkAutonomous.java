package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name = "Back Parking Autonomous", group = "Autonomous")
public class BackParkAutonomous extends BaseAutonomous {
    @Override
    public void start() {
        super.start();

        move(1.2);
        sleep(250);
        move(-0.1);
    }
}
