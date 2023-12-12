package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

/** A static class containing all useful tools for various hardware interfaces. */
public class Utils {

    /**
     *  Reset that motor's encoder count - a count that tells how much
     * that specific motor has moved. Finding out the Motor's pulse per
     * revolution, we can find out it's total revolution through : <br>
     *
     * CURRENT_MOTOR_POSITION / PULSES_PER_REVOLUTION = TOTAL_REVOLUTION
     * */
    public static void resetMotorEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
