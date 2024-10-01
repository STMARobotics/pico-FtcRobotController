/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.BooleanSupplier;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

public class DriveSubsystem {

    public static final String IMU = "imu";
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public static final String FRONT_LEFT_MOTOR = "front_left_motor";
    public static final String FRONT_RIGHT_MOTOR = "front_right_motor";
    public static final String BACK_LEFT_MOTOR = "back_left_motor";
    public static final String BACK_RIGHT_MOTOR = "back_right_motor";

    private double countsPerMotorRev;
    private double countsPerInch;
    private double driveGearReduction;     // No External Gearing.

    final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private BooleanSupplier isActiveOpMode;
    private int newFrontRightTarget;
    private int newFrontLeftTarget;
    private int newBackRightTarget;
    private int newBackLeftTarget;
    private IMU imu;


    public DriveSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.hardwareMap = hm;
        this.telemetry = telemetry;
        this.init(hm);
        countsPerMotorRev = frontLeftMotor.getMotorType().getTicksPerRev();
        driveGearReduction = frontLeftMotor.getMotorType().getGearing();
        countsPerInch = (countsPerMotorRev * driveGearReduction) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
    }
    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    protected void init(HardwareMap hm) {
        setupIMU();

        this.hardwareMap = hm;

        assignMotors();

        assignDriveDirections();

        shutOffMotors();

        driveWithoutEncoders();
    }

    private void setupIMU() {
        imu = hardwareMap.get(IMU.class, IMU);
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    public void resetYaw(){
        imu.resetYaw();
    }

    public void driveWithoutEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shutOffMotors() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    private void setPower(double value) {
        frontRightMotor.setPower(value);
        frontLeftMotor.setPower(value);
        backRightMotor.setPower(value);
        backLeftMotor.setPower(value);
    }

    private void assignDriveDirections() {
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    private void assignMotors() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR);
        frontRightMotor = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR);
        backLeftMotor = hardwareMap.get(DcMotor.class, BACK_LEFT_MOTOR);
        backRightMotor = hardwareMap.get(DcMotor.class, BACK_RIGHT_MOTOR);
    }

    public void moveRobotCentric(float forward, float strafe, float turn, float reductionFactor) {
            float originalDenominator = calculateDenominator(forward / reductionFactor, strafe / reductionFactor, turn / reductionFactor);

            float adjustedDenominator = originalDenominator * reductionFactor;
            float frontLeftPower = (forward + strafe + turn) / adjustedDenominator;
            float backLeftPower = (forward - strafe + turn) / adjustedDenominator;
            float frontRightPower = (forward - strafe - turn) / adjustedDenominator;
            float backRightPower = (forward + strafe - turn) / adjustedDenominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.addData("Reduction Factor", reductionFactor);
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);

    }

    public void moveFieldCentric(double x, double y, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
    }

    public void setPower(float forward, float strafe, float turn, float reductionFactor) {
        float originalDenominator = calculateDenominator(forward/reductionFactor, strafe/reductionFactor, turn/reductionFactor);

        float adjustedDenominator = originalDenominator * reductionFactor;
        float frontLeftPower = (forward + strafe + turn) / adjustedDenominator;
        float backLeftPower = (forward - strafe + turn) / adjustedDenominator;
        float frontRightPower = (forward - strafe - turn) / adjustedDenominator;
        float backRightPower = (forward + strafe - turn) / adjustedDenominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.addData("Reduction Factor", reductionFactor);
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Back Right Power", backRightPower);
    }

    private float calculateDenominator(float forward, float strafe, float turn) {
        float sum = Math.abs(forward) + Math.abs(strafe) + Math.abs(turn);
        if (sum > 1) {
            return sum;
        }
        return 1;
    }

    public void driveWithEncoders() {

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at", "FL: %7d FR: %7d BL: %7d BR: %7d",
                frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(),
                backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());

    }

    public void driveForward(double speed, double inches) {

        newFrontRightTarget = frontRightMotor.getCurrentPosition() + (int) (inches * countsPerInch);
        newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + (int) (inches * countsPerInch);
        newBackRightTarget = backRightMotor.getCurrentPosition() + (int) (inches * countsPerInch);
        newBackLeftTarget = backLeftMotor.getCurrentPosition() + (int) (inches * countsPerInch);

        frontRightMotor.setTargetPosition(newFrontRightTarget);
        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        backRightMotor.setTargetPosition(newBackRightTarget);
    }

    public void strafeLeft(double speed, double inches) {

        int newFrontRightTarget = frontRightMotor.getCurrentPosition() + (int) (inches * countsPerInch);
        int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + (int) (inches * countsPerInch);
        int newBackRightTarget = backRightMotor.getCurrentPosition() - (int) (inches * countsPerInch);
        int newBackLeftTarget = backLeftMotor.getCurrentPosition() - (int) (inches * countsPerInch);

        frontRightMotor.setTargetPosition(newFrontRightTarget);
        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        backRightMotor.setTargetPosition(newBackRightTarget);
    }

    public void turnLeft(double speed, double degrees) {

        int newFrontRightTarget = frontRightMotor.getCurrentPosition() + (int) (degrees * countsPerInch);
        int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() - (int) (degrees * countsPerInch);
        int newBackRightTarget = backRightMotor.getCurrentPosition() + (int) (degrees * countsPerInch);
        int newBackLeftTarget = backLeftMotor.getCurrentPosition() - (int) (degrees * countsPerInch);

        frontRightMotor.setTargetPosition(newFrontRightTarget);
        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        backRightMotor.setTargetPosition(newBackRightTarget);

    }

    public boolean isMoving() {
        return frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy() && backLeftMotor.isBusy();
    }

    public void logPosition(){
        telemetry.addData("Running to", "FL: %7d FR: %7d BL: %7d BR: %7d",
                newFrontLeftTarget, newFrontRightTarget,
                newBackLeftTarget, newBackRightTarget);
        telemetry.addData("Currently at", "FL: %7d FR: %7d BL: %7d BR: %7d",
                frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(),
                backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        telemetry.update();
    }
}
