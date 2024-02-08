package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Range;

import org.firstinspires.ftc.teamcode.all_purpose.HardwareManager;
import org.firstinspires.ftc.teamcode.all_purpose.Misc;


// danny's recording test
// so basically im trying to track movements by detecting when any button is pressed. the recordArray 2D array will store two key value pairs (?)
//

public abstract class RecordingTest extends OpMode {
    //protected ArrayList<String> recordArray;
    protected boolean dPadPressed = false;

    protected ElapsedTime timeElapsed;

    protected HardwareManager hardwareManager;

    protected double frontLeftWheelPower;
    protected double frontRightWheelPower;
    protected double backLeftWheelPower;
    protected double backRightWheelPower;

    protected double bottomArmMotorPower;
    protected double topArmServoPower;
    protected double clawServoPower;

    protected double liftMotorPower;

    protected boolean unhook;

    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------
    // Power range [MINIMUM, MAXIMUM]
    protected static final Range<Double> WHEELS_POWER_RANGE    = new Range<>(-1.0, 1.0);
    protected static final Range<Double> ARM_MOTOR_POWER_RANGE = new Range<>(-1.0, 1.0);
    protected static final Range<Double> ARM_SERVO_POWER_RANGE = new Range<>(-0.8, 0.8);
    protected static final Range<Double> LIFT_POWER_RANGE      = new Range<>(-1.0, 0.5);
    protected static final double LAUNCHER_BASE_POSITION = 0.5;

    //------------------------------------------------------------------------------------------------
    // Defaults
    //------------------------------------------------------------------------------------------------
    protected static final double GAME_PAD_TRIGGER_THRESHOLD = 0.5;
    protected static final double LIFT_POWER_DELTA = 1.0;
    protected static final double ARM_MOTOR_DELTA = 1.0;
    protected static final double ARM_SERVO_DELTA = 1.0;
    /*
    public void processUserInputTest() {

        if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_up) {
            if (!buttonPressed) {
                buttonPressed = true;
                timeElapsed.reset();
            }
            if (gamepad1.dpad_right) {
                        setLeftPower(1);
                setRightPower(-1);
            } else if (gamepad1.dpad_left) {
                setLeftPower(-1);
                setRightPower(1);
            } else if (gamepad1.dpad_up) {
                setLeftPower(-1);
                setRightPower(-1);
            } else if (gamepad1.dpad_down) {
                setLeftPower(1);
                setRightPower(1);
            }
        } else {

            if (buttonPressed) {
                buttonPressed = false;
                recordArray.app
            }
            setLeftPower(0);
            setRightPower(0);
        }



    }

     */

    public void setRightPower(double power) {
        frontRightWheelPower = power;
        backRightWheelPower = power;
    }

    public void setLeftPower(double power) {
        frontLeftWheelPower = power;
        backLeftWheelPower = power;
    }

    protected void useDefaultArmControls() {
        bottomArmMotorPower = 0;
        topArmServoPower = 0;
        clawServoPower = 0;

        // Bottom Arm
        if (gamepad1.left_bumper)
            bottomArmMotorPower = ARM_MOTOR_DELTA;
        else if (gamepad1.left_trigger > GAME_PAD_TRIGGER_THRESHOLD)
            bottomArmMotorPower = -ARM_MOTOR_DELTA;

        // Top Arm
        if (gamepad1.right_bumper)
            topArmServoPower = ARM_SERVO_DELTA;
        else if (gamepad1.right_trigger > GAME_PAD_TRIGGER_THRESHOLD)
            topArmServoPower = -ARM_SERVO_DELTA;

        // Claw
        if (gamepad1.dpad_right)
            clawServoPower = ARM_SERVO_DELTA;
        else if (gamepad1.dpad_left)
            clawServoPower = -ARM_SERVO_DELTA;
    }

    protected void useDefaultLiftControls() {
        liftMotorPower = 0;

        if (gamepad1.dpad_up || gamepad2.dpad_up)
            liftMotorPower = LIFT_POWER_DELTA;
        else if (gamepad1.dpad_down || gamepad2.dpad_down)
            liftMotorPower = -LIFT_POWER_DELTA;
    }

    protected void useDefaultDroneLauncherControls() {
        unhook = gamepad1.start || gamepad2.start;
    }

    //------------------------------------------------------------------------------------------------
    // nEw ArM cONtrOls (experimental)
    //------------------------------------------------------------------------------------------------
    protected final double COUNTS_PER_ELBOW_REVOLUTION = 1440;
    protected final double COUNTS_PER_ANGLE = COUNTS_PER_ELBOW_REVOLUTION / 360.0; // DEGREES
    protected final double ELBOW_MOTOR_POWER = 0.5;
    protected boolean isUp = false;
    protected void moveElbowMotors(double endPositionAngle) {

        /*
         * This is a copy of the moveElbowMotors() from selfDriving with a very very
         * experimental way of changing the direction of the motor
         * */
        if(endPositionAngle >= 0) {
            hardwareManager.elbowArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            hardwareManager.elbowArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        double total_counts = COUNTS_PER_ANGLE * endPositionAngle; // Angle of movement desired
        double motor_position = hardwareManager.elbowArmMotor.getCurrentPosition(); // Elbow's current position
        double target_angle = (endPositionAngle >= 0)
                ? (motor_position + total_counts)
                : (motor_position - total_counts); // The desired end position of the motors angle

        hardwareManager.elbowArmMotor.setPower(ELBOW_MOTOR_POWER);
        while (hardwareManager.elbowArmMotor.getCurrentPosition() <= target_angle) {
            //idle()
            ;        }

        hardwareManager.elbowArmMotor.setPower(0);
    }
    protected void useSimpleArmControls(){
        if (!hardwareManager.elbowArmMotor.isBusy()) {
            if(gamepad1.x){
                telemetry.update();
                if(!isUp){
                    moveElbowMotors(-45);
                    isUp = false;
                    // if x is pressed then arm moves down by 45 degrees
                }else{
                    moveElbowMotors(45);
                    isUp = true;
                    // if y is pressed then arm moves up by 45 degrees
                }
            }
        }

        if(gamepad1.a){
            moveClawServos(true);
        } else if(gamepad1.b){
            moveClawServos(false);
        }
        /*
         * this is still a big work in progress
         * This code works!!
         */
    }
    protected void armControlsOverride(){

    }

    //---- Wrist Controls ----
    protected boolean rightServoBusy = false;
    protected boolean leftServoBusy = false;
    protected int CLAW_OPEN_MS = 0;

    protected void setServoIsBusy(boolean isServoOne, boolean isBusy){
        if(isServoOne){
            leftServoBusy = isBusy;
        } else {
            rightServoBusy = isBusy;
        }
    }
    protected void moveClawServos (boolean isServoOne){
//        boolean motorStateBusy = (isServoOne)
//                ? leftServoBusy
//                : rightServoBusy;
//        if(!motorStateBusy){
//            CRServoImplEx selectedServo = (isServoOne)
//                    ?hardwareManager.clawServoLeft
//                    :hardwareManager.clawServoRight;
//
//            switch(selectedServo.getDirection()){
//                case FORWARD:
//                    selectedServo.setDirection(DcMotorSimple.Direction.REVERSE);
//                    break;
//                default:
//                    selectedServo.setDirection(DcMotorSimple.Direction.FORWARD);
//                    break;
//            }
//
//            selectedServo.setPower(0.7);
//            setServoIsBusy(isServoOne, true);
//            sleep(CLAW_OPEN_MS);
//            setServoIsBusy(isServoOne, false);
//            selectedServo.setPower(0);
//        }

    }

    //------------------------------------------------------------------------------------------------
    // Debug
    //------------------------------------------------------------------------------------------------

    protected void checkSpeedOfEachWheel(int currentTime, int changeInSeconds){
        if(currentTime % changeInSeconds == 0){

        }

    }

    public void sleep(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e){}
    }

    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------

    protected abstract void processUserInput();

    @Override
    public void loop() {
        processUserInput();

        hardwareManager.getFrontLeftWheel().setPower(WHEELS_POWER_RANGE.clamp(frontLeftWheelPower));
        hardwareManager.getFrontRightWheel().setPower(WHEELS_POWER_RANGE.clamp(frontRightWheelPower));
        hardwareManager.getBackLeftWheel().setPower(WHEELS_POWER_RANGE.clamp(backLeftWheelPower));
        hardwareManager.getBackRightWheel().setPower(WHEELS_POWER_RANGE.clamp(backRightWheelPower));

//        hardwareManager.clawServoLeft.setPower(ARM_SERVO_POWER_RANGE.clamp(clawServoPower));
//        hardwareManager.clawServoRight.setPower(ARM_SERVO_POWER_RANGE.clamp(clawServoPower));

        hardwareManager.liftMotor.setPower(LIFT_POWER_RANGE.clamp(liftMotorPower));

        //hardwareManager.droneLauncherBase.setPosition(0);
        //hardwareManager.droneLauncherHook.setPosition(0);

        //if (unhook)
            //hardwareManager.droneLauncherHook.setPosition(1);
    }

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);
    }

    @Override
    public void start() {
        //hardwareManager.droneLauncherBase.setPosition(LAUNCHER_BASE_POSITION);
    }

}
