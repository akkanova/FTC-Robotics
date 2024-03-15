package org.firstinspires.ftc.teamcode.tests.debug;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.tests.BaseTest;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;

/** Records each and every button presses from a Gamepad */
public class RecordGamepad extends BaseTest {
    public static final File RECORD_DIR = new File(AppUtil.FIRST_FOLDER, "/recoded-op-modes/");
    public static final String VERSION = "v.0.0.1|";

    public static final long sampleRate = 100; // Each line is 100ms

    @Override
    public void runOpMode() {
        AppUtil.getInstance().ensureDirectoryExists(RECORD_DIR);
        waitForStart();

        ElapsedTime elapsedTime = new ElapsedTime();
        AppUtil.getInstance().getModalContext();

        try (
            FileOutputStream fileOutputStream = new FileOutputStream(new File(RECORD_DIR, "test-file"));
            PrintWriter printWriter = new PrintWriter(fileOutputStream)
        ) {
            printWriter.println(VERSION.concat(Long.toString(sampleRate)));
            while (opModeIsActive() && elapsedTime.seconds() <= 30) {
                printWriter.println(new String(gamepad1.toByteArray(), StandardCharsets.UTF_8));
                sleep(sampleRate);
            }

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
