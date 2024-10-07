package org.firstinspires.ftc.teamcode;

public interface Command {
    void execute();

    void init();

    boolean isFinished();

    void logMessage();

    void onComplete();

    int getTimeout();
}
