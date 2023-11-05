package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name = "Left Start Autonomous", group = "Autonomous")
public class LeftStartAutonomous extends BaseAutonomous {
    @Override
    public void start() {
        super.start();

        move(1.8);
        sleep(250);
        rotate(-90);
    }
}