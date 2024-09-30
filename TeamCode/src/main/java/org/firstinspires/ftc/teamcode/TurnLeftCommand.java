package org.firstinspires.ftc.teamcode;

public class TurnLeftCommand extends DriveCommand{


    private double speed;
    private final double degrees;

    public TurnLeftCommand(DriveSubsystem driveSystem, double degrees, int timeout) {
        this.driveSystem = driveSystem;
        this.degrees = degrees;
        this.timeout = timeout;
    }

    @Override
    public void execute() {
        driveSystem.turnLeft(speed, degrees);
    }
}
