package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.MecanumLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.tests.debug.DriveStraightDebug;
import org.firstinspires.ftc.teamcode.tests.debug.GamepadDebug;
import org.firstinspires.ftc.teamcode.tests.debug.RecordGamepad;
import org.firstinspires.ftc.teamcode.tests.debug.VisionProcessorDebug;
import org.firstinspires.ftc.teamcode.tests.road_runner.LocalizationTest;
import org.firstinspires.ftc.teamcode.tests.road_runner.ManualFeedbackTuner;
import org.firstinspires.ftc.teamcode.tests.road_runner.SplineTest;
import org.firstinspires.ftc.teamcode.tests.tuning.ElbowPIDFTuner;
import org.firstinspires.ftc.teamcode.tests.tuning.ElbowPositionFinder;
import org.firstinspires.ftc.teamcode.tests.tuning.ServoPositionTuner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Registers all test OpModes */
public final class TestOpModesRegistrar {
    private TestOpModesRegistrar() {};

    @OpModeRegistrar
    public static void register(OpModeManager opModeManager) {
        if (!GlobalConfig.DISABLE_ROAD_RUNNER_TUNING)
            registerRoadRunnerTuningOpModes(opModeManager);

        if (!GlobalConfig.DISABLE_HARDWARE_TUNING_OP_MODES)
            registerHardwareTuningOpModes(opModeManager);

        if (!GlobalConfig.DISABLE_DEBUG_OP_MODES)
            registerDevelopmentDebugOpModes(opModeManager);
    }

    /** Debug Op Modes used during development */
    private static void registerDevelopmentDebugOpModes(OpModeManager opModeManager) {
        String group = "development-debug";

        opModeManager.register(getMetaForClass(DriveStraightDebug.class, group), new DriveStraightDebug());
        opModeManager.register(getMetaForClass(VisionProcessorDebug.class, group), new VisionProcessorDebug());
        opModeManager.register(getMetaForClass(RecordGamepad.class, group), new RecordGamepad());
        opModeManager.register(getMetaForClass(GamepadDebug.class, group), new GamepadDebug());
    }

    /** Tuning Op Modes designed for general hardware calibration */
    private static void registerHardwareTuningOpModes(OpModeManager opModeManager) {
        String group = "hardware-tuning";

        opModeManager.register(getMetaForClass(ElbowPIDFTuner.class, group), new ElbowPIDFTuner());
        opModeManager.register(getMetaForClass(ElbowPositionFinder.class, group), new ElbowPositionFinder());
        opModeManager.register(getMetaForClass(ServoPositionTuner.class, group), new ServoPositionTuner());

        allowConfigurationWithFtcDashboard(
            ServoPositionTuner.class
        );
    }

    /**
     * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tuning/TuningOpModes.java">
     *     Extracted from Road Runner quickstart  - TuningOpModes.java
     * </a>
     */
    private static void registerRoadRunnerTuningOpModes(OpModeManager opModeManager) {
        DriveViewFactory driveViewFactory = hardwareMap -> {
            HardwareManager hardwareManager = new HardwareManager(hardwareMap);
            MecanumDrive mecanumDrive = new MecanumDrive(hardwareManager, new Pose2d(0, 0, 0));

            List<Encoder> leftEncoders = new ArrayList<>();
            List<Encoder> rightEncoders = new ArrayList<>();
            List<Encoder> parallelEncoders = new ArrayList<>();
            List<Encoder> perpendicularEncoders = new ArrayList<>();

            if (mecanumDrive.localizer instanceof MecanumLocalizer) {
                MecanumLocalizer localizer = (MecanumLocalizer) mecanumDrive.localizer;
                leftEncoders.add(localizer.frontLeftWheelEncoder);
                rightEncoders.add(localizer.frontRightWheelEncoder);
                leftEncoders.add(localizer.backLeftWheelEncoder);
                rightEncoders.add(localizer.backRightWheelEncoder);

            } else if (mecanumDrive.localizer instanceof ThreeWheelLocalizer) {
                ThreeWheelLocalizer localizer = (ThreeWheelLocalizer) mecanumDrive.localizer;
                parallelEncoders.add(localizer.leftParallelEncoder);
                parallelEncoders.add(localizer.rightParallelEncoder);
                perpendicularEncoders.add(localizer.perpendicularEncoder);

            } else {
                throw new RuntimeException("Unsupported localizer " + mecanumDrive.localizer.getClass().getName());
            }

            return new DriveView(
                DriveType.MECANUM,
                GlobalConfig.MecanumDriveConfig.inchesPerTick,
                GlobalConfig.MecanumDriveConfig.maxWheelVelocity,
                GlobalConfig.MecanumDriveConfig.minProfileAcceleration,
                GlobalConfig.MecanumDriveConfig.maxProfileAcceleration,
                hardwareMap.getAll(LynxModule.class),
                Arrays.asList(
                    hardwareManager.getFrontLeftWheelMotor(),
                    hardwareManager.getBackLeftWheelMotor()
                ),
                Arrays.asList(
                    hardwareManager.getFrontRightWheelMotor(),
                    hardwareManager.getBackRightWheelMotor()
                ),
                leftEncoders, rightEncoders,
                parallelEncoders, perpendicularEncoders,
                hardwareManager.lazyIMU,
                hardwareManager.batteryVoltageSensor,
                () -> new MotorFeedforward(
                    GlobalConfig.MecanumDriveConfig.kS,
                    GlobalConfig.MecanumDriveConfig.kV / GlobalConfig.MecanumDriveConfig.inchesPerTick,
                    GlobalConfig.MecanumDriveConfig.kA / GlobalConfig.MecanumDriveConfig.inchesPerTick
                )
            );
        };

        String group = "road-runner-tuning";

        opModeManager.register(getMetaForClass(AngularRampLogger.class, group), new AngularRampLogger(driveViewFactory));
        opModeManager.register(getMetaForClass(ForwardPushTest.class, group), new ForwardPushTest(driveViewFactory));
        opModeManager.register(getMetaForClass(ForwardRampLogger.class, group), new ForwardRampLogger(driveViewFactory));
        opModeManager.register(getMetaForClass(LateralPushTest.class, group), new LateralPushTest(driveViewFactory));
        opModeManager.register(getMetaForClass(LateralRampLogger.class, group), new LateralRampLogger(driveViewFactory));
        opModeManager.register(getMetaForClass(ManualFeedforwardTuner.class, group), new ManualFeedforwardTuner(driveViewFactory));
        opModeManager.register(getMetaForClass(MecanumMotorDirectionDebugger.class, group), new MecanumMotorDirectionDebugger(driveViewFactory));
        opModeManager.register(getMetaForClass(DeadWheelDirectionDebugger.class, group), new DeadWheelDirectionDebugger(driveViewFactory));

        // All the road runner tuning op modes not included in the library
            opModeManager.register(getMetaForClass(ManualFeedbackTuner.class, group), new ManualFeedbackTuner());
        opModeManager.register(getMetaForClass(SplineTest.class, group), new SplineTest());
        opModeManager.register(getMetaForClass(LocalizationTest.class, group), new LocalizationTest());

        // Allow them to be configurable in FTC-Dashboard
        allowConfigurationWithFtcDashboard(
            LocalizationTest.class,
            AngularRampLogger.class,
            ForwardRampLogger.class,
            LateralRampLogger.class,
            ManualFeedbackTuner.class,
            ManualFeedforwardTuner.class,
            MecanumMotorDirectionDebugger.class
        );
    }

    /** Add additional description and info needed by {@link  OpModeManager} */
    private static OpModeMeta getMetaForClass(Class<? extends OpMode> opModeClass, String group) {
        return new OpModeMeta.Builder()
            .setName(opModeClass.getSimpleName())
            .setFlavor(OpModeMeta.Flavor.TELEOP)
            .setGroup(group)
            .build();
    }

    /**
     * Allow the inputted op modes to be configurable using FTC-Dashboard. This way allows
     * them to be dynamically added in, as opposed to using the {@link Config} class attribute.
     * */
    @SafeVarargs
    private static void allowConfigurationWithFtcDashboard(Class<? extends OpMode> ...opModeClasses) {
        FtcDashboard
            .getInstance()
            .withConfigRoot((configRoot) -> {
                for (Class<? extends  OpMode> clazz : opModeClasses) {
                    configRoot.putVariable(
                        clazz.getSimpleName(),
                        ReflectionConfig.createVariableFromClass(clazz)
                    );
                }
            });
    }
}
