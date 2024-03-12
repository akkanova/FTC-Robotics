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

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.common.GlobalConfig;
import org.firstinspires.ftc.teamcode.common.HardwareManager;
import org.firstinspires.ftc.teamcode.common.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.MecanumLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.localizers.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.tests.debug.DriveStraightDebug;
import org.firstinspires.ftc.teamcode.tests.debug.GamepadDebug;
import org.firstinspires.ftc.teamcode.tests.debug.VisionProcessorDebug;
import org.firstinspires.ftc.teamcode.tests.debug.WheelVelocitiesDebug;
import org.firstinspires.ftc.teamcode.tests.tuning.LocalizationTest;
import org.firstinspires.ftc.teamcode.tests.tuning.ManualFeedbackTuner;
import org.firstinspires.ftc.teamcode.tests.tuning.SplineTest;

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

        if (!GlobalConfig.DISABLE_DEBUG_OP_MODES)
            registerDevelopmentDebugOpModes(opModeManager);
    }

    /** Debug Op Modes used during development */
    private static void registerDevelopmentDebugOpModes(OpModeManager opModeManager) {
        String group = "development-debug";

        opModeManager.register(getMetaForClass(GamepadDebug.class, group), new GamepadDebug());
        opModeManager.register(getMetaForClass(DriveStraightDebug.class, group), new DriveStraightDebug());
//        opModeManager.register(getMetaForClass(WheelVelocitiesDebug.class, group), new WheelVelocitiesDebug());
        opModeManager.register(getMetaForClass(VisionProcessorDebug.class, group), new VisionProcessorDebug());
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

            // TODO : add support for a three wheel localizer

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

        // TODO : add later ..
        opModeManager.register(getMetaForClass(ManualFeedbackTuner.class, group), new ManualFeedbackTuner());
        opModeManager.register(getMetaForClass(SplineTest.class, group), new SplineTest());
        opModeManager.register(getMetaForClass(LocalizationTest.class, group), new LocalizationTest());

        // Allow them to be configurable in FTC-Dashboard
        FtcDashboard
            .getInstance()
            .withConfigRoot((configRoot) -> {
                for (Class<?> clazz : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedbackTuner.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class
                )) {
                    configRoot.putVariable(
                        clazz.getSimpleName(),
                        ReflectionConfig.createVariableFromClass(clazz)
                    );
                }
            });
    }

    private static OpModeMeta getMetaForClass(Class<? extends OpMode> opModeClass, String group) {
        return new OpModeMeta.Builder()
            .setName(opModeClass.getSimpleName())
            .setFlavor(OpModeMeta.Flavor.TELEOP)
            .setGroup(group)
            .build();
    }
}
