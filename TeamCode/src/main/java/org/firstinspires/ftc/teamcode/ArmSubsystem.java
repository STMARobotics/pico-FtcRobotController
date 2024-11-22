package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ArmSubsystem {
    private DcMotor armMotor;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public ArmSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.hardwareMap = hm;
        this.telemetry = telemetry;
        this.init(hm);}
    protected void init(HardwareMap hm) {

        this.hardwareMap = hm;

        assignMotors();

        assignDriveDirections();

        shutOffMotors();

        driveWithoutEncoders();
    }

    public static final String ARM_MOTOR = "arm_motor";

    public void driveWithoutEncoders() {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shutOffMotors() {
        armMotor.setPower(0);
    }

    public void setPower(double arm) {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(arm);
    }

    private void assignDriveDirections() {
        armMotor.setDirection(DcMotor.Direction.FORWARD);
    }


    private void assignMotors() {
        armMotor = hardwareMap.get(DcMotor.class, ARM_MOTOR);
    }


    public void stop() {
        int currentArm =  armMotor.getCurrentPosition();
        armMotor.setTargetPosition(currentArm);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(.5);
    }
}