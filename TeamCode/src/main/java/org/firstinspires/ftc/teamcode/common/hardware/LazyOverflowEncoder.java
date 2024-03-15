package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.misc.Lazy;

/** Lazy loaded OverflowEncoder; initializes only when needed. */
public class LazyOverflowEncoder extends Lazy<OverflowEncoder> {
    /** Create an instance that automatically assumes its using the correct direction*/
    public LazyOverflowEncoder(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap, deviceName, false);
    }

    /** Create an instance with the ability to flip the direction */
    public LazyOverflowEncoder(
        HardwareMap hardwareMap,
        String deviceName,
        boolean flipDirection
    ) {
        super(() -> {
            DcMotorEx motorEx = hardwareMap.get(DcMotorEx.class, deviceName);
            OverflowEncoder encoder = new OverflowEncoder(new RawEncoder(motorEx));

            if (flipDirection)
                encoder.setDirection(DcMotorSimple.Direction.REVERSE);

            // Reset Positions
            DcMotor.RunMode mode = motorEx.getMode();
            motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorEx.setMode(mode);

            return encoder;
        });
    }
}
