package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristSubsystem {
    public static final String WRIST_SERVO = "wrist-servo";

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Servo servo;

    public WristSubsystem(HardwareMap hm, Telemetry telemetry){
        this.hardwareMap = hm;
        this.telemetry = telemetry;

        servo = hardwareMap.get(Servo.class, WRIST_SERVO);
    }

    public void moveToPosition(double position){
        servo.setPosition(position);
    }
}
