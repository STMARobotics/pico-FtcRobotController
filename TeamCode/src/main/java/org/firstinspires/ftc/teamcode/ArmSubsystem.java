package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ArmSubsystem {
    private DcMotor armMotor;
    private CRServo intakeServo;
    private Servo leverLock;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public ArmSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.hardwareMap = hm;
        this.telemetry = telemetry;
        this.init(hm);
    }

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
        leverLock = hardwareMap.get(Servo.class, "leverLock");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
    }





    public void stop() {
        int currentArm =  armMotor.getCurrentPosition();
        armMotor.setTargetPosition(currentArm);
        armMotor.setPower(.5);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void logPosition(){
        telemetry.addData("armMotor",armMotor.getCurrentPosition());
    }
    public void intake(){
        intakeServo.setPower(-1);
    }
    public void drop(){
        intakeServo.setPower(1);
    }


    public void unlock(){leverLock.setPosition(0);}
    public void lock(){leverLock.setPosition(.25);}
}