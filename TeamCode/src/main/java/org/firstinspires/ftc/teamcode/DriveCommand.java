package org.firstinspires.ftc.teamcode;

public abstract class DriveCommand implements Command{
    protected DriveSubsystem driveSystem;
    protected int timeout;

    @Override
    public boolean isFinished() {
        return !driveSystem.isMoving();
    }

    @Override
    public void logMessage() {
        driveSystem.logPosition();
    }

    @Override
    public void onComplete() {
        driveSystem.shutOffMotors();
    }

    @Override
    public int getTimeout() {
        return this.timeout;
    }

    @Override
    public void init(){
        driveSystem.driveWithEncoders();
    }
}
