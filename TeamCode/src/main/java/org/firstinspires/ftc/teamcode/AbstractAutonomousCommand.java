package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AbstractAutonomousCommand {
    protected LinearOpMode opMode = null;
    protected void waitUntilDone(int timeoutSeconds) {
        ElapsedTime runtime = new ElapsedTime();
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutSeconds) && ! isFinished()){
            logMessage();
        }
        onComplete();
    }

    protected void onComplete(){}

    protected void logMessage(){}

    abstract boolean isFinished();
}
