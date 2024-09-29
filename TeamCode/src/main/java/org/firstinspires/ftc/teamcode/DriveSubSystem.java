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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

public class DriveSubSystem {

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor frontLeftMotor   = null;
    private DcMotor frontRightMotor  = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public static final String FRONT_LEFT_MOTOR = "front-left-motor";
    public static final String FRONT_RIGHT_MOTOR = "front-right-motor";
    public static final String BACK_LEFT_MOTOR = "back-left-motor";
    public static final String BACK_RIGHT_MOTOR = "back-right-motor";


    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap hm, Telemetry telemetry)    {
        this.hardwareMap = hm;
        this.telemetry = telemetry;

        assignMotors();

        motorDirections();

        setPower(0);

        runWithoutEncoder();
    }

    private void runWithoutEncoder() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setPower(double value) {
        frontLeftMotor.setPower(value);
        frontRightMotor.setPower(value);
        backLeftMotor.setPower(value);
        backRightMotor.setPower(value);
    }

    private void motorDirections() {
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void assignMotors() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR);
        frontRightMotor = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR);
        backLeftMotor = hardwareMap.get(DcMotor.class, BACK_LEFT_MOTOR);
        backRightMotor = hardwareMap.get(DcMotor.class, BACK_RIGHT_MOTOR);
    }

    public void setPower(float forward, float strafe, float turn, float reductionFactor) {
        float originalDenominator = calculateDeniminator(forward/reductionFactor, strafe/reductionFactor, turn/reductionFactor);
        float adjustedDenominator = originalDenominator * reductionFactor;
        float frontLeftPower = (forward + strafe + turn ) / adjustedDenominator;
        float frontRightPower = (forward - strafe - turn ) / adjustedDenominator;
        float backLeftPower = (forward - strafe + turn ) / adjustedDenominator;
        float backRightPower = (forward + strafe - turn ) / adjustedDenominator;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe",strafe);
        telemetry.addData("Turn",turn);
        telemetry.addData("Reduction Factor",reductionFactor);
        telemetry.addData("Front Left power" , frontLeftPower);
        telemetry.addData("Front Right power", frontRightPower);
        telemetry.addData("Back Left power", backLeftPower);
        telemetry.addData("Back Right power", backRightPower);





    }
    private float calculateDeniminator(float forward, float strafe, float turn) {
        float sum = Math.abs(forward) + Math.abs(strafe) + Math.abs(turn);
        if (sum > 1){
            return sum;
        }
        return 1;
    }
}
