package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.SelfDriving;

@Autonomous(name = "Pick up and drop", group = "Autonomous")
public class PixelPickUpAndDrop extends SelfDriving {
    @Override
    protected void runAutonomous(){
        moveElbowMotor(-80);
        sleep(2000);
        openClawServos(false);
        openClawServos(true);
        sleep(2000);
        moveElbowMotor(80);
    }
}
