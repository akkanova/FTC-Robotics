package org.firstinspires.ftc.teamcode.common.hardware;

import android.util.Range;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Consumer;

/**
 * A class for communicating with specific wheel & motor arrangements (such as Tank & Mecanum Drive)
 * to facilitate specific robot translations and rotations.
 */
public class Drivetrain {
    public abstract static class Base {
        protected Range<Double> powerRange = new Range<>(-1.0, 1.0);

        /** Set the Lower and Upper limits of the power provided to the motors. */
        public void setPowerRange(double lower, double upper) {
            setPowerRange(new Range<>(lower, upper));
        }

        /** Set the bounds to the power provided to the motors. */
        public void setPowerRange(Range<Double> range) {
            this.powerRange = range;
        }

        /** @return The current power limit bounds */
        public Range<Double> getPowerRange() {
            return powerRange;
        }

        // INHERITANCE
        abstract void resetAllMotorEncoders();
        abstract void stop();
    }


    //----------------------------------------------------------------------------------------------
    // Mecanum Drivetrain
    //----------------------------------------------------------------------------------------------

    /** For a drive base with a 4 Mecanum Wheels Configuration. */
    public static class Mecanum extends Base {
        private final DcMotor[] motors;

        /**
         * Create an instance with the provided Qualcomm DcMotor interfaces.
         * This assumes all provided motors are already configured to the correct
         * direction.
         *
         * @param frontLeftMotor   front-left motor
         * @param frontRightMotor  front-right motor
         * @param backLeftMotor    back-left motor
         * @param backRightMotor   back-right motor
         *  */
        public Mecanum(
                DcMotor frontLeftMotor,
                DcMotor frontRightMotor,
                DcMotor backLeftMotor,
                DcMotor backRightMotor
        ) {
            this(new DcMotor[] {
                    frontLeftMotor, frontRightMotor,
                    backLeftMotor,  backRightMotor
            });
        }

        /**
         *  Create an instance with the provided motors array.
         * @param motors an array that follows this sequence of locations :
         *               Front-Left, Front-Right, Back-Left, & Back-Right.
         *  */
        public Mecanum(DcMotor[] motors) {
            this.motors = motors;
        }

        /** Run the callback for each of the 4 Motors. */
        public void doForAllWheels(Consumer<DcMotor> callback) {
            for (DcMotor motor : motors)
                callback.accept(motor);
        }

        /**
         * Move to that direction with the provided power and heading angle.
         * @param headingPower -1 to 1 - Heading motor power (-1 backward, 1 forward).
         * @param headingAngle Radian - Angle referring to the heading of the robot,
         *                     relative to the drivetrain center.
         * @param rotate -1 to 1 - Rotation power (-1 Spin left, 1 Spin right).
         */
        public void drive(double headingPower, double headingAngle, double rotate) {
            double sin = Math.sin(headingAngle - Math.PI / 4);
            double cos = Math.cos(headingAngle - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double frontLeftPower  = headingPower * cos / max + rotate;
            double frontRightPower = headingPower * sin / max - rotate;
            double backLeftPower   = headingPower * sin / max + rotate;
            double backRightPower  = headingPower * cos / max - rotate;

            if ((headingPower + Math.abs(rotate)) > 1) {
                frontLeftPower  /= headingPower + rotate;
                frontRightPower /= headingPower + rotate;
                backLeftPower   /= headingPower + rotate;
                backRightPower  /= headingPower + rotate;
            }

            motors[0].setPower(powerRange.clamp(frontLeftPower));
            motors[1].setPower(powerRange.clamp(frontRightPower));
            motors[2].setPower(powerRange.clamp(backLeftPower));
            motors[3].setPower(powerRange.clamp(backRightPower));
        }

        /**
         * Reset all the encoder count for all the motors - a count that tells how much
         * that specific motor has moved. Finding out the Motor's pulse per
         * revolution, we can find out it's total revolution through : <br>
         *
         * CURRENT_MOTOR_POSITION / PULSES_PER_REVOLUTION = TOTAL_REVOLUTION
         * */
        @Override
        void resetAllMotorEncoders() {
            doForAllWheels(Utils::resetMotorEncoder);
        }

        /** Set the power of all the motors to 0. */
        @Override
        public void stop() {
            doForAllWheels(wheel -> wheel.setPower(0));
        }


        // GETTERS

        /** @return The front left motor. */
        public DcMotor getFrontLeftMotor() {
            return motors[0];
        }

        /** @return The front Right motor. */
        public DcMotor getFrontRightMotor() {
            return motors[1];
        }

        /** @return The back left motor. */
        public DcMotor getBackLeftMotor() {
            return motors[2];
        }

        /** @return The back right motor. */
        public DcMotor getBackRightMotor() {
            return motors[3];
        }
    }


    //----------------------------------------------------------------------------------------------
    // Tank Drivetrain
   //----------------------------------------------------------------------------------------------

    /** For a drive base that uses differential power for steering and movement. */
    public static class Tank extends Base {
        private final DcMotor[] leftMotors;
        private final DcMotor[] rightMotors;

        /**
         * Create an instance with any number of motors on each side. This assumes all
         * provided motors are already configured to the correct direction.
         *
         * @param leftMotors  All the motors located on the left side of the drivetrain.
         * @param rightMotors All the motors located on the right side of the drivetrain.
         * */
        public Tank(
                DcMotor[] leftMotors,
                DcMotor[] rightMotors
        ) {
            this.leftMotors  = leftMotors;
            this.rightMotors = rightMotors;
        }

        /** Run the callback for each of the motors located on the left side. */
        public void doForAllLeftSideMotors(Consumer<DcMotor> callback) {
            for (DcMotor motor : leftMotors)
                callback.accept(motor);
        }

        /** Run the callback for each of the motors located on the right side. */
        public void doForAllRightSideMotors(Consumer<DcMotor> callback) {
            for (DcMotor motor : rightMotors)
                callback.accept(motor);
        }

        /**
         * Drive via power differential. (-1 backward, 1 forward)
         * @param leftPower -1 to 1 - Power for the entire left side.
         * @param rightPower -1 to 1 - Power for the entire right side.
         */
        public void drive(double leftPower, double rightPower) {
            doForAllLeftSideMotors(motor  -> motor.setPower(powerRange.clamp(leftPower)));
            doForAllRightSideMotors(motor -> motor.setPower(powerRange.clamp(rightPower)));
        }

        /**
         * Reset all the encoder count for all the motors - a count that tells how much
         * that specific motor has moved. Finding out the Motor's pulse per
         * revolution, we can find out it's total revolution through : <br>
         *
         * CURRENT_MOTOR_POSITION / PULSES_PER_REVOLUTION = TOTAL_REVOLUTION
         * */
        @Override
        public void resetAllMotorEncoders() {
            doForAllLeftSideMotors(Utils::resetMotorEncoder);
            doForAllRightSideMotors(Utils::resetMotorEncoder);
        }

        /** Set the power of all the motors to 0. */
        @Override
        public void stop() {
            doForAllLeftSideMotors(motor  -> motor.setPower(0));
            doForAllRightSideMotors(motor -> motor.setPower(0));
        }


        // GETTERS

        /** @return all motors on the left side */
        public DcMotor[] getLeftSideMotors() {
            return leftMotors;
        }

        /** @return all motors on the right side */
        public DcMotor[] getRightSideMotors() {
            return leftMotors;
        }

    }
}
