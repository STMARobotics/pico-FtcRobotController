package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutonomousDrivingCommand {
    private final LinearOpMode opMode;
    private final DriveSubSystem driveSystem;

    public AutonomousDrivingCommand(LinearOpMode opMode, DriveSubSystem driveSubSystem) {
        this.opMode = opMode;
        this.driveSystem = driveSubSystem;
    }

    public void driveForward(double speed, double distanceInInches, int timeoutSeconds) {
        driveSystem.driveWithEncoders();

        driveSystem.driveForward(speed, distanceInInches);

        waitUntilDone(timeoutSeconds);
    }

    public void strafeLeft(double speed, double distanceInInches, int timeoutSeconds) {
        driveSystem.driveWithEncoders();

        driveSystem.strafeLeft(speed, distanceInInches);

        waitUntilDone(timeoutSeconds);
    }

    public void turnLeft(double speed, double degrees, int timeoutSeconds) {
        driveSystem.driveWithEncoders();

        driveSystem.turnLeft(speed, degrees);

        waitUntilDone(timeoutSeconds);
    }

    private void waitUntilDone(int timeoutSeconds) {
        ElapsedTime runtime = new ElapsedTime();
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutSeconds) && driveSystem.isMoving()){
            driveSystem.logPosition();
        }
    }
}
