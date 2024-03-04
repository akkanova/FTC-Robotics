package org.firstinspires.ftc.teamcode.common.misc;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.common.GlobalConfig;

/** Utility class for additional FTC-Dashboard helper methods */
public final class DashboardUtils {
    /**
     * Draws a circle and a line to represent the space the robot currently occupies and
     * direction it's current heading.

     *  <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Drawing.java">
     *      Extracted from Road Runner quickstart  -  Drawing.java
     *  </a>
     * */
    public static void drawRobot(Canvas canvas, Pose2d currentPose) {
        canvas.setStrokeWidth(1);
        canvas.strokeCircle(
                currentPose.position.x,
                currentPose.position.y,
                GlobalConfig.MecanumDriveConfig.robotRadius
        );

        Vector2d directionVector = currentPose.heading.vec()
                .times(0.5 * GlobalConfig.MecanumDriveConfig.robotRadius);

        Vector2d p1 = currentPose.position.plus(directionVector);
        Vector2d p2 = p1.plus(directionVector);
        canvas.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}
