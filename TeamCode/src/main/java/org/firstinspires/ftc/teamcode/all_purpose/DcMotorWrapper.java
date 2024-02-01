package org.firstinspires.ftc.teamcode.all_purpose;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DcMotorWrapper extends DcMotorImplEx {
    private double lastMotorPosition;
    private double lastCheckedTimestamp;
    private ElapsedTime elapsedTime;

    public DcMotorWrapper(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
        elapsedTime = new ElapsedTime();
    }

    public float getSpeed() {
        float deltaTime = (float) (elapsedTime.seconds() - lastCheckedTimestamp);
        float speed = (float) (getCurrentPosition() - lastMotorPosition) / deltaTime;
        lastMotorPosition = getCurrentPosition();
        lastCheckedTimestamp = elapsedTime.milliseconds();
        return speed;
    }
}
