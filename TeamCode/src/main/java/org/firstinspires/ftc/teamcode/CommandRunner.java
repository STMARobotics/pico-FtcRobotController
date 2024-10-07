package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CommandRunner {

    private final LinearOpMode opMode;
    private Command[] commands;

    public CommandRunner(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public CommandRunner commands(Command... commands) {
        this.commands = commands;
        return this;
    }

    public static CommandRunner OpMode(LinearOpMode opMode) {
        return new CommandRunner(opMode);
    }

    public void run (){
        for (Command command : commands) {
            command.init();
            command.execute();
            ElapsedTime runtime = new ElapsedTime();
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < command.getTimeout()) && ! command.isFinished()){
                command.logMessage();
            }
            command.onComplete();
        }
    }
}
