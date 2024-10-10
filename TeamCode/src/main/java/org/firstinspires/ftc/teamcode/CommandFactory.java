package org.firstinspires.ftc.teamcode;

public class CommandFactory {
    private static CommandFactory commandFactory;
    private final DriveSubsystem driveSubsystem;

    private CommandFactory(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    public static void InitFactory(DriveSubsystem driveSubSystem){
        commandFactory = new CommandFactory(driveSubSystem);
    }


    public static Command Forward(double inchesForward, double speed, int timeout){
       return new ForwardCommand(commandFactory.driveSubsystem, inchesForward, speed, timeout);
    }

    public static Command Backward(double inchesBackward, double speed, int timeout){
        return new ForwardCommand(commandFactory.driveSubsystem, -inchesBackward, speed, timeout);
    }

    public static Command StrafeLeft(double inchesLeft, double speed, int timeout){
        return new StrafeLeftCommand(commandFactory.driveSubsystem, inchesLeft, speed, timeout);
    }

    public static Command StrafeRight (double inchesRight, double speed, int timeout){
        return new StrafeLeftCommand(commandFactory.driveSubsystem, -inchesRight, speed, timeout);
    }

    public static Command TurnLeft(double degrees, int timeout){
        return new TurnLeftCommand(commandFactory.driveSubsystem, degrees, timeout);
    }

    public static Command TurnRight(double degrees, int timeout){
        return new TurnLeftCommand(commandFactory.driveSubsystem, -degrees, timeout);
    }
}
