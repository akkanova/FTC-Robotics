package org.firstinspires.ftc.teamcode.common.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.TimeTurn;

import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.MecanumLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.misc.DashboardUtils;
import org.firstinspires.ftc.teamcode.common.misc.RoadRunnerLog;

import java.util.LinkedList;
import java.util.List;

/**
 * Road Runner drive base implementation for
 * an all mecanum wheels assembly.

 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java">
 *     Based on Road Runner quickstart  -  MecanumDrive.java
 * </a>
 * */
public final class MecanumDrive {
    private final RoadRunnerLog.MecanumDriveLogger logger;
    private final HardwareManager hardwareManager;
    private final LinkedList<Pose2d> poseHistory;

    public final Localizer localizer;
    public Pose2d currentPose;

    /**
     * Create an instance of MecanumDrive that utilizes provided localizer.
     * */
    public MecanumDrive(
        HardwareManager hardwareManager,
        Pose2d initialPose,
        Localizer localizer
    ) {
        this.logger = new RoadRunnerLog.MecanumDriveLogger();
        this.hardwareManager = hardwareManager;
        this.poseHistory = new LinkedList<>();
        this.currentPose = initialPose;
        this.localizer = localizer;

        // A: I know the original logged the parameters, which is the global config, but
        // the config has to have STATIC variables for it to be dynamically
        // configurable through an FTC Dashboard..
        FlightRecorder.write("MECANUM_DRIVE_INITIALIZED", System.nanoTime());
    }

    /**
     * Create an instance of MecanumDrive that utilizes the default wheel
     * encoders for localization.
     * */
    public MecanumDrive(HardwareManager hardwareManager, Pose2d initialPose) {
        this(hardwareManager, initialPose, new ThreeWheelLocalizer(hardwareManager));
    }

    /**
     * Sets the calculated necessary individual wheel power to achieve the
     * inputted position and velocity.
     * */
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVelocities = new MecanumKinematics(1)
            .inverse(PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMagnitude = 1;
        for (DualNum<Time> power : wheelVelocities.all()) {
            maxPowerMagnitude = Math.max(maxPowerMagnitude, power.value());
        }

        hardwareManager.getFrontLeftWheelMotor().setPower(
            wheelVelocities.leftFront.get(0) / maxPowerMagnitude);
        hardwareManager.getFrontRightWheelMotor().setPower(
            wheelVelocities.rightFront.get(0) / maxPowerMagnitude);
        hardwareManager.getBackLeftWheelMotor().setPower(
            wheelVelocities.leftBack.get(0) / maxPowerMagnitude);
        hardwareManager.getBackRightWheelMotor().setPower(
            wheelVelocities.rightBack.get(0) / maxPowerMagnitude);
    }

    /**
     * Updates the current pose with a new one provided by the current localizer.
     * @return the current pose and velocity.
     * */
    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        currentPose = currentPose.plus(twist.value());
        poseHistory.add(currentPose);

        // Prune pose history if too large
        while (poseHistory.size() > GlobalConfig.MecanumDriveConfig.maxPoseHistory) {
            poseHistory.removeFirst();
        }

        logger.estimatedPoseWriter.write(new RoadRunnerLog.PoseLogMessage(currentPose));
        return twist.velocity().value();
    }

    /**
     * @return an instance of {@link TrajectoryActionBuilder} configured with the
     * necessary options to facilitate actions with this mecanum drive base.
     */
    public TrajectoryActionBuilder getNewActionBuilder(Pose2d initialPose) {
        return new TrajectoryActionBuilder(
            TurnAction::new,
            FollowTrajectoryAction::new,
            initialPose, 1e-6, 0,
            GlobalConfig.MecanumDriveConfig.defaultTurnConstraints,
            GlobalConfig.MecanumDriveConfig.defaultVelocityConstraint,
            GlobalConfig.MecanumDriveConfig.defaultAccelerationConstraint,
            0.25, 0.1
        );
    }

    private void drawPoseHistory(Canvas canvas) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        for (int i = 0; i < poseHistory.size(); i++) {
            xPoints[i] = poseHistory.get(i).position.x;
            yPoints[i] = poseHistory.get(i).position.y;
        }

        canvas.setStrokeWidth(1);
        canvas.setStroke("#3F51B5");
        canvas.strokePolyline(xPoints, yPoints);
    }

    //-----------------------------------------------------------------------------------
    // Actions
    //-----------------------------------------------------------------------------------

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory timeTrajectory) {
            this.timeTrajectory = timeTrajectory;
            List<Double> paths = com.acmerobotics.roadrunner.Math.range(
                    0, timeTrajectory.path.length(),
                    Math.max(2, (int) Math.ceil(timeTrajectory.path.length() / 2))
            );

            xPoints = new double[paths.size()];
            yPoints = new double[paths.size()];

            for (int i = 0; i < paths.size(); i++) {
                Pose2d pose = timeTrajectory.path.get(paths.get(i), 1).value();
                xPoints[i] = pose.position.x;
                yPoints[i] = pose.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double time;
            if (beginTs < 0) {
                beginTs = Actions.now();
                time = 0;
            } else {
                time = Actions.now() - beginTs;
            }

            if (time >= timeTrajectory.duration) {
                hardwareManager.getFrontLeftWheelMotor().setPower(0);
                hardwareManager.getFrontRightWheelMotor().setPower(0);
                hardwareManager.getBackLeftWheelMotor().setPower(0);
                hardwareManager.getBackRightWheelMotor().setPower(0);
                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(time);
            logger.targetPoseWriter.write(new RoadRunnerLog.PoseLogMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            PoseVelocity2dDual<Time> command = new HolonomicController(
                    GlobalConfig.MecanumDriveConfig.axialGain,
                    GlobalConfig.MecanumDriveConfig.lateralGain,
                    GlobalConfig.MecanumDriveConfig.headingGain,
                    GlobalConfig.MecanumDriveConfig.axialVelocityGain,
                    GlobalConfig.MecanumDriveConfig.lateralVelocityGain,
                    GlobalConfig.MecanumDriveConfig.headingVelocityGain
            ).compute(txWorldTarget, currentPose, robotVelRobot);
            logger.driveCommandWriter.write(new RoadRunnerLog.DriveCommandLogMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVelocities =
                    GlobalConfig.MecanumDriveConfig.kinematics.inverse(command);

            double voltage = hardwareManager.batteryVoltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(
                    GlobalConfig.MecanumDriveConfig.kS,
                    GlobalConfig.MecanumDriveConfig.kV / GlobalConfig.MecanumDriveConfig.inchesPerTick,
                    GlobalConfig.MecanumDriveConfig.kA / GlobalConfig.MecanumDriveConfig.inchesPerTick
            );

            double frontLeftPower  = feedforward.compute(wheelVelocities.leftFront)  / voltage;
            double frontRightPower = feedforward.compute(wheelVelocities.rightFront) / voltage;
            double backLeftPower   = feedforward.compute(wheelVelocities.leftBack)  / voltage;
            double backRightPower  = feedforward.compute(wheelVelocities.rightBack) / voltage;
            logger.mecanumCommandWriter.write(new RoadRunnerLog.MecanumCommandLogMessage(
                    voltage, frontLeftPower, frontRightPower, backLeftPower, backRightPower
            ));

            hardwareManager.getFrontLeftWheelMotor().setPower(frontLeftPower);
            hardwareManager.getFrontRightWheelMotor().setPower(frontRightPower);
            hardwareManager.getBackLeftWheelMotor().setPower(backLeftPower);
            hardwareManager.getBackRightWheelMotor().setPower(backRightPower);

            packet.put("x", currentPose.position.x);
            packet.put("y", currentPose.position.y);
            packet.put("heading (deg)", Math.toDegrees(currentPose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(currentPose);
            packet.put("xError", error.position.x);
            packet.put("yError", error.position.y);
            packet.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas canvas = packet.fieldOverlay();
            drawPoseHistory(canvas);

            canvas.setStroke("#4CAF50");
            DashboardUtils.drawRobot(canvas, txWorldTarget.value());

            canvas.setStroke("#3F51B5");
            DashboardUtils.drawRobot(canvas, currentPose);
            canvas.setStroke("#4CAF50FF");
            canvas.setStrokeWidth(1);
            canvas.strokePolyline(xPoints, yPoints);
            return true;
        }

        @Override
        public void preview(Canvas canvas) {
            canvas.setStroke("#4CAF507A");
            canvas.setStrokeWidth(1);
            canvas.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;
        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double time;
            if (beginTs < 0) {
                beginTs = Actions.now();
                time = 0;
            } else {
                time = Actions.now() - beginTs;
            }

            if (time >= turn.duration) {
                hardwareManager.getFrontLeftWheelMotor().setPower(0);
                hardwareManager.getFrontRightWheelMotor().setPower(0);
                hardwareManager.getBackLeftWheelMotor().setPower(0);
                hardwareManager.getBackRightWheelMotor().setPower(0);
                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(time);
            logger.targetPoseWriter.write(new RoadRunnerLog.PoseLogMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            PoseVelocity2dDual<Time> command = new HolonomicController(
                GlobalConfig.MecanumDriveConfig.axialGain,
                GlobalConfig.MecanumDriveConfig.lateralGain,
                GlobalConfig.MecanumDriveConfig.headingGain,
                GlobalConfig.MecanumDriveConfig.axialVelocityGain,
                GlobalConfig.MecanumDriveConfig.lateralVelocityGain,
                GlobalConfig.MecanumDriveConfig.headingVelocityGain
            ).compute(txWorldTarget, currentPose, robotVelRobot);
            logger.driveCommandWriter.write(new RoadRunnerLog.DriveCommandLogMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVelocities =
                GlobalConfig.MecanumDriveConfig.kinematics.inverse(command);

            double voltage = hardwareManager.batteryVoltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(
                    GlobalConfig.MecanumDriveConfig.kS,
                    GlobalConfig.MecanumDriveConfig.kV / GlobalConfig.MecanumDriveConfig.inchesPerTick,
                    GlobalConfig.MecanumDriveConfig.kA / GlobalConfig.MecanumDriveConfig.inchesPerTick
            );

            double frontLeftPower = feedforward.compute(wheelVelocities.leftFront) / voltage;
            double frontRightPower = feedforward.compute(wheelVelocities.rightFront) / voltage;
            double backLeftPower = feedforward.compute(wheelVelocities.leftBack) / voltage;
            double backRightPower = feedforward.compute(wheelVelocities.rightBack) / voltage;
            logger.mecanumCommandWriter.write(new RoadRunnerLog.MecanumCommandLogMessage(
                    voltage, frontLeftPower, frontRightPower, backLeftPower, backRightPower
            ));

            hardwareManager.getFrontLeftWheelMotor().setPower(frontLeftPower);
            hardwareManager.getFrontRightWheelMotor().setPower(frontRightPower);
            hardwareManager.getBackLeftWheelMotor().setPower(backLeftPower);
            hardwareManager.getBackRightWheelMotor().setPower(backRightPower);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            DashboardUtils.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            DashboardUtils.drawRobot(c, currentPose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas canvas) {
            canvas.setStroke("#7C4DFF7A");
            canvas.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }
}
