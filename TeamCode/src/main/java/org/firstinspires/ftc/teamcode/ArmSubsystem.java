package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ArmSubsystem {
    private DcMotor armMotor;
    private Servo intakeServo;
    private Servo wrist;
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
        wrist = hardwareMap.get(Servo.class, "wristServo");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
    }
    public void overTheWall(){
        armMotor.setTargetPosition(100);
        armMotor.setPower(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void basketDrop(){
        armMotor.setTargetPosition(10);
        armMotor.setPower(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void stop() {
        int currentArm =  armMotor.getCurrentPosition();
        armMotor.setTargetPosition(currentArm);
        armMotor.setPower(.5);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void movetoposition(int position){
        armMotor.setTargetPosition(position);
        armMotor.setPower(.5);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean isMoving(){
        return armMotor.isBusy();
    }

    public void logPosition(){
        telemetry.addData("armMotor",armMotor.getCurrentPosition());
    }

    public void intake(){
        intakeServo.setPosition(1);
    }
    public void drop(){intakeServo.setPosition(.68);}
    public void logservoPosition(){
      //  telemetry.addData("intakeServo",intakeServo.getCurrentPosition());
    }
    public void up(){wrist.setPosition(1);}
    public void down(){wrist.setPosition(.5);}
}