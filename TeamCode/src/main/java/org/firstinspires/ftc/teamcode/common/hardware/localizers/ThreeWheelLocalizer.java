package org.firstinspires.ftc.teamcode.common.hardware.localizers;

import static org.firstinspires.ftc.teamcode.common.GlobalConfig.ThreeWheelLocalizerConfig.inchesPerTick;
import static org.firstinspires.ftc.teamcode.common.GlobalConfig.ThreeWheelLocalizerConfig.leftParallelYTicks;
import static org.firstinspires.ftc.teamcode.common.GlobalConfig.ThreeWheelLocalizerConfig.perpendicularXTicks;
import static org.firstinspires.ftc.teamcode.common.GlobalConfig.ThreeWheelLocalizerConfig.rightParallelYTicks;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.misc.RoadRunnerLog;

/**
 * Localizer that utilizes the three wheels odometer kit. Consisting of two parallel
 * and one parallel dead-wheel encoders.

 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ThreeDeadWheelLocalizer.java">
 *      Copied from Road Runner quickstart  -  ThreeDeadWheelLocalizer.java
 * </a>
 */
public class ThreeWheelLocalizer implements Localizer {
    public final Encoder leftParallelEncoder;
    public final Encoder rightParallelEncoder;
    public final Encoder perpendicularEncoder;

    private int lastLeftParallelPosition;
    private int lastRightParallelPosition;
    private int lastPerpendicularPosition;
    private boolean initialized;

    public ThreeWheelLocalizer(HardwareManager hardwareManager) {
        leftParallelEncoder  = hardwareManager.deadWheelLeftEncoder.get();
        rightParallelEncoder = hardwareManager.deadWheelRightEncoder.get();
        perpendicularEncoder = hardwareManager.deadWheelPerpendicularEncoder.get();

        FlightRecorder.write("THREE_DEAD_WHEEL_INITIALIZED", System.nanoTime());
    }

    @Override
    public Twist2dDual<Time> update() {
        PositionVelocityPair leftParallelPosVel  = leftParallelEncoder.getPositionAndVelocity();
        PositionVelocityPair rightParallelPosVel = rightParallelEncoder.getPositionAndVelocity();
        PositionVelocityPair perpendicularPosVel = perpendicularEncoder.getPositionAndVelocity();

        FlightRecorder.write(
            "THREE_DEAD_WHEEL_INPUTS",
            new RoadRunnerLog.ThreeDeadWheelInputsLogMessage(
                leftParallelPosVel,
                rightParallelPosVel,
                perpendicularPosVel
            )
        );

        if (!initialized) {
            initialized = true;

            lastLeftParallelPosition  = leftParallelPosVel.position;
            lastRightParallelPosition = rightParallelPosVel.position;
            lastPerpendicularPosition = perpendicularPosVel.position;

            return new Twist2dDual<>(
                Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                DualNum.constant(0.0, 2)
            );
        }

        int leftParallelPosDelta  = leftParallelPosVel.position  - lastLeftParallelPosition;
        int rightParallelPosDelta = rightParallelPosVel.position - lastRightParallelPosition;
        int perpendicularPosDelta = perpendicularPosVel.position - lastPerpendicularPosition;

        Twist2dDual<Time> twist = new Twist2dDual<>(
            new Vector2dDual<>(
                new DualNum<Time>(new double[] {
                    (leftParallelYTicks * rightParallelPosDelta - rightParallelYTicks * leftParallelPosDelta) / (leftParallelYTicks - rightParallelYTicks),
                    (leftParallelYTicks * rightParallelPosVel.velocity - rightParallelYTicks * leftParallelPosVel.velocity) / (leftParallelYTicks - rightParallelYTicks),
                }).times(inchesPerTick),
                new DualNum<Time>(new double[]{
                    (perpendicularXTicks / (leftParallelYTicks - rightParallelYTicks) * (rightParallelPosDelta - leftParallelPosDelta) + perpendicularPosDelta),
                    (perpendicularXTicks / (leftParallelYTicks - rightParallelYTicks) * (rightParallelPosVel.velocity - leftParallelPosVel.velocity) + perpendicularPosVel.velocity)
                }).times(inchesPerTick)
            ),
            new DualNum<>(new double[] {
                (leftParallelPosDelta - rightParallelPosDelta) / (leftParallelYTicks - rightParallelYTicks),
                (leftParallelPosVel.velocity - rightParallelPosVel.velocity) /(leftParallelYTicks - rightParallelYTicks)
            })
        );

        lastLeftParallelPosition  = leftParallelPosVel.position;
        lastRightParallelPosition = rightParallelPosVel.position;
        lastPerpendicularPosition = perpendicularPosVel.position;

        return twist;
    }
}
