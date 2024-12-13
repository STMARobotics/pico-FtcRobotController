package org.firstinspires.ftc.teamcode;



/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.BooleanSupplier;

public class SlideSubsystem {

    public static final String IMU = "imu";
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor slideLeftMotor;
    private DcMotor slideRightMotor;
    private Servo basketMotor;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public static final String SLIDE_LEFT_MOTOR = "slide_left_motor";
    public static final String SLIDE_RIGHT_MOTOR = "slide_right_motor";
    public static final String BASKET_MOTOR = "basket_servo";

    private double countsPerMotorRev;
    private double countsPerInch;
    private double driveGearReduction;     // No External Gearing.


    public SlideSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.hardwareMap = hm;
        this.telemetry = telemetry;
        this.init(hm);
        countsPerMotorRev = slideLeftMotor.getMotorType().getTicksPerRev();
        driveGearReduction = slideLeftMotor.getMotorType().getGearing();
    }
    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    protected void init(HardwareMap hm) {

        this.hardwareMap = hm;


        assignMotors();

        setToZero();

        assignDriveDirections();

        shutOffMotors();

        driveWithoutEncoders();

    }


    public void driveWithoutEncoders() {
        slideLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shutOffMotors() {
        slideRightMotor.setPower(0);
        slideLeftMotor.setPower(0);
    }
    private void setToZero(){
        slideLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPower(double slide) {
        slideRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRightMotor.setPower(slide);
        slideLeftMotor.setPower(slide);
    }

    private void assignDriveDirections() {
        slideLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        slideRightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private void assignMotors() {
        slideLeftMotor = hardwareMap.get(DcMotor.class, SLIDE_LEFT_MOTOR);
        slideRightMotor = hardwareMap.get(DcMotor.class, SLIDE_RIGHT_MOTOR);
        basketMotor = hardwareMap.get(Servo.class, BASKET_MOTOR);

    }

    public void moveRobotCentric(float slide) {
        float slideLeftPower = slide;
        float slideRightPower = slide;

        slideLeftMotor.setPower(slideLeftPower);
        slideRightMotor.setPower(slideRightPower);


        telemetry.addData("slide",slide );
        telemetry.addData("slide Right Power",slideRightPower );
        telemetry.addData("slide Left Power",slideLeftPower );

    }
    public void runToPosition(int position) {
        slideLeftMotor.setTargetPosition(position);
        slideRightMotor.setTargetPosition(position);
        slideLeftMotor.setPower(.5);
        slideRightMotor.setPower(.5);
        slideLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public int getPosition(){
        int currentLeft =  slideLeftMotor.getCurrentPosition();
        int currentRight =  slideRightMotor.getCurrentPosition();
        return (currentLeft+currentRight)/2;
    }


    public void stop() {
        int currentLeft =  slideLeftMotor.getCurrentPosition();
        int currentRight =  slideRightMotor.getCurrentPosition();
        slideLeftMotor.setTargetPosition(currentLeft);
        slideRightMotor.setTargetPosition(currentRight);
        slideLeftMotor.setPower(.5);
        slideRightMotor.setPower(.5);
        slideLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void logPosition(){
        telemetry.addData("slideMotorPosition",getPosition());
    }
    public void climb1(){
        slideLeftMotor.setTargetPosition(1000);
        slideRightMotor.setTargetPosition(1000);
        slideLeftMotor.setPower(.5);
        slideRightMotor.setPower(.5);
        slideLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeftMotor.setTargetPosition(0);
        slideRightMotor.setTargetPosition(0);
        slideLeftMotor.setPower(.5);
        slideRightMotor.setPower(.5);
        slideLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void climb2(){
        slideLeftMotor.setTargetPosition(1000);
        slideRightMotor.setTargetPosition(0);
        slideLeftMotor.setPower(.5);
        slideRightMotor.setPower(.5);
        slideLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeftMotor.setTargetPosition(0);
        slideRightMotor.setTargetPosition(0);
        slideLeftMotor.setPower(.5);
        slideRightMotor.setPower(.5);
        slideLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void basketdump(){
        basketMotor.setPosition(.25);
    }
    public void basketup(){
        basketMotor.setPosition(0);
    }

}

