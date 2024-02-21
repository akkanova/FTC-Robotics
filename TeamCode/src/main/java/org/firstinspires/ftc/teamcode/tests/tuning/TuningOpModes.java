package org.firstinspires.ftc.teamcode.tests.tuning;

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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * <a href="https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tuning/TuningOpModes.java">
 *      Copied from Road Runner quickstart  -  TuningOpModes.java
 * </a>. Modified for mecanum drive base testing only.
 */
public final class TuningOpModes {
    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setGroup(GROUP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED)
            return;

        DriveViewFactory driveViewFactory = (hardwareMap) -> {
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

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(driveViewFactory));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(driveViewFactory));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(driveViewFactory));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(driveViewFactory));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(driveViewFactory));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(driveViewFactory));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(driveViewFactory));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(driveViewFactory));

        // TODO : add later..
//        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
//        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        FtcDashboard.getInstance().withConfigRoot((configRoot) -> {
            for (Class<?> c : Arrays.asList(
                AngularRampLogger.class,
                ForwardRampLogger.class,
                LateralRampLogger.class,
                ManualFeedforwardTuner.class,
                MecanumMotorDirectionDebugger.class
//               ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(
                    c.getSimpleName(),
                    ReflectionConfig.createVariableFromClass(c)
                );
            }
        });
    }
}