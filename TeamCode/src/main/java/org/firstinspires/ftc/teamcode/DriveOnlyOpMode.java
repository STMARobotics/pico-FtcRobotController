/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "Drive Only OpMode", group = "Linear OpMode")
//@Disabled
public class DriveOnlyOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DriveSubsystem driveSubsystem;
    SlideSubsystem slideSubsystem;
    ArmSubsystem armSubsystem;

    @Override
    public void runOpMode() {
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        slideSubsystem = new SlideSubsystem(hardwareMap, telemetry);
        armSubsystem = new ArmSubsystem(hardwareMap, telemetry);
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        float forward;
        float strafe;
        float turn;
        float reductionFactor;
        while (opModeIsActive()) {

            forward = -gamepad1.left_stick_y;
            float slide = gamepad2.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            float arm = gamepad2.right_stick_y;


            double fudgeFactorPercentage = gamepad2.right_trigger + (-gamepad2.left_trigger);

            reductionFactor = 1;
            if (gamepad1.left_bumper) {
                reductionFactor = 4;
            }
            Servo intakeServo = hardwareMap.get(Servo.class, "intakeServo");
            //Claw driving
            if (gamepad2.a) {
                armSubsystem.intake();
            }
            else if (gamepad2.y) {
                armSubsystem.drop();
            }
            //wrist driving
            if (gamepad2.dpad_down) {
                armSubsystem.lock();
            } else if (gamepad2.dpad_up) {
                armSubsystem.unlock();
            }
            //bucket driving
            if (gamepad2.b){
                slideSubsystem.basketdump();
            }
            if(gamepad2.x){
                slideSubsystem.basketup();
            }

            //Arm driving
            if (arm > .1) {
                armSubsystem.setPower(-arm);
            } else if (Math.abs(arm) > .1) {

                armSubsystem.setPower(-arm);

            } else {
                armSubsystem.stop();

                armSubsystem.logPosition();

                //Commands for slide subsystem
                if (gamepad1.a) {
                    slideSubsystem.climb1();
                }
                else if (gamepad1.y){
                    slideSubsystem.climb2();}
                else if (gamepad2.b) {
                    slideSubsystem.basketdump();}
                     else if (Math.abs(slide) > .1) {
                        slideSubsystem.setPower(-slide);}
                else  {
                    slideSubsystem.stop();
                }



                // set target postion to current position
                // change mode to run to encoder
                // set power on again


                // Move Slide to positions

                armSubsystem.logPosition();
                slideSubsystem.logPosition();
                telemetry.update();


                driveSubsystem.moveRobotCentric(forward, strafe, turn, reductionFactor);
            }

            // Show the elapsed game time and wheel power.


        }
    }
}
