package org.firstinspires.ftc.teamcode;

public class StrafeLeftCommand extends DriveCommand{
    private final double inchesForward;
    private final double speed;

    public StrafeLeftCommand(DriveSubsystem driveSystem, double inchesForward, double speed, int timeout) {
        this.driveSystem = driveSystem;
        this.inchesForward = inchesForward;
        this.speed = speed;
        this.timeout = timeout;
    }

    @Override
    public void execute() {
        driveSystem.strafeLeft(speed, inchesForward);
    }
}
