package org.firstinspires.ftc.teamcode.common.hardware.localizers;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

/**
 * Base interface implemented by all any road-runner localizer (built-in drive encoders, two wheel,
 * or three wheel odometers) implementations..<br>

 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Localizer.java" >
 *    Copied from Road Runner quickstart  -  Localizer.java
 * </a>
 */
public interface Localizer {
    Twist2dDual<Time> update();
}
