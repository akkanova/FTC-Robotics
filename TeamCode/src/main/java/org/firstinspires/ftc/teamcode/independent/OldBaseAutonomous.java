package org.firstinspires.ftc.teamcode.independent;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;

public abstract class OldBaseAutonomous extends LinearOpMode {
    protected HardwareManager hardwareManager;
    protected MecanumDrive drive;
    protected IMU imu;

    // How much can the autonomous allow for deviations
    protected static double FORWARD_LENIENCY = 1;
    protected static double TURN_LENIENCY = 0;
    protected static double TURN_POWER = 0.5;
    protected static double MOVE_POWER = 0.5;

    protected abstract void runAuto();

    @Override
    public void runOpMode() throws InterruptedException {
      // Initial position doesn't matter.
        hardwareManager = new HardwareManager(hardwareMap);
        drive = new MecanumDrive(hardwareManager, new Pose2d(0, 0, 0));

        waitForStart();
        runAuto();
    }

    public void move(double inchesDistance) {
        if (!opModeIsActive())
            return;

        double initialPosition = drive.currentPose.position.x + inchesDistance;

        while(opModeIsActive() && Math.abs(drive.currentPose.position.x - initialPosition) > FORWARD_LENIENCY) {
            drive.updatePoseEstimate();
            drive.setDrivePowers(
                 new PoseVelocity2d(
                     new Vector2d(
                         MOVE_POWER, 0
                     ), 0
                 )
            );
        }

        sleep(250);
    }

    // return inches

    public void rotate(double degreeAngle) {
        if (!opModeIsActive())
            return;

        imu.resetYaw();
        double initialAngle = getCurrentDegreeHeading();

        double motorOffset = degreeAngle > 0 ? 1 : -1;
        double leftPower = TURN_POWER * motorOffset;
        double rightPower = TURN_POWER * -motorOffset;

        hardwareManager.getFrontRightWheelMotor().setPower(leftPower);
        hardwareManager.getFrontRightWheelMotor().setPower(rightPower);
        hardwareManager.getBackLeftWheelMotor().setPower(leftPower);
        hardwareManager.getBackRightWheelMotor().setPower(rightPower);

        while (opModeIsActive() && hasReachedDesiredAngle(initialAngle, degreeAngle)) {
            // idle();
        }

        hardwareManager.doForAllWheels(wheel -> wheel.setPower(0));
        sleep(250);
    }

    public double getCurrentDegreeHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    protected boolean hasReachedDesiredAngle(double initialAngle, double turnAngle) {
        double targetAngle = initialAngle - turnAngle;
        double currentAngle = getCurrentDegreeHeading();

        return turnAngle > 0
            ? currentAngle >= targetAngle - TURN_LENIENCY
            : currentAngle <= targetAngle + TURN_LENIENCY;
    }
}
