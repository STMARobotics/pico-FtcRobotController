package org.firstinspires.ftc.teamcode;

public class ForwardCommand extends DriveCommand implements Command{
    private final double inchesForward;
    private final double speed;

    public ForwardCommand(DriveSubsystem driveSystem, double inchesForward, double speed, int timeout) {
        this.driveSystem = driveSystem;
        this.inchesForward = inchesForward;
        this.speed = speed;
        this.timeout = timeout;
    }

    @Override
    public void execute() {
        driveSystem.driveForward(speed, inchesForward);
    }
}