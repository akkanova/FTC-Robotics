package org.firstinspires.ftc.teamcode.independent;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LeftAudienceStart", group = "Autonomous")
public class LeftAudienceStart extends OldBaseAutonomous {
    @Override
    protected void runAuto() {
        move(55.1181);
        rotate(-90);
        move(102.362);
        rotate(-45);
    }
}
