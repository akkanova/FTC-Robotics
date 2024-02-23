package org.firstinspires.ftc.teamcode.common.hardware.localizers;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

/**
 * Base interface implemented by all any Road Runner localizer.
 * Which includes the three-wheel odometers and mecanum drive-base localizer.<br>

 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Localizer.java" >
 *    Copied from Road Runner quickstart  -  Localizer.java
 * </a>
 */
public interface Localizer {
    Twist2dDual<Time> update();
}
