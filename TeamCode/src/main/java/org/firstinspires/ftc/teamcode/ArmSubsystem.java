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

    public static final String ARM_MOTOR = "arm_motor";

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT              = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER        = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN       = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW  = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK  = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT          = 10  * ARM_TICKS_PER_DEGREE;

    static double FUDGE_FACTOR_DEGREES = 15;

    public ArmSubsystem(HardwareMap hm, Telemetry telemetry)    {
        this.hardwareMap = hm;
        this.telemetry = telemetry;

        armMotor=hm.get(DcMotor.class, ARM_MOTOR);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx)armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveToArmToCollectPosition(double alteration){

        setArmPosition(ARM_COLLECT + getAlterationDegrees(alteration));
    }

    private double getAlterationDegrees(double alteration) {
        return FUDGE_FACTOR_DEGREES * ARM_TICKS_PER_DEGREE * alteration;
    }

    public void moveArmToClearBarrierPosition(double alteration){
        setArmPosition(ARM_CLEAR_BARRIER + (getAlterationDegrees(alteration)));
    }
    public void moveArmToScoreSampleInLowPosition(double alteration){
        setArmPosition(ARM_SCORE_SAMPLE_IN_LOW + (getAlterationDegrees(alteration)));
    }
    public void moveArmToCollapsedIntoRobotPosition(double alteration){
        setArmPosition(ARM_COLLAPSED_INTO_ROBOT + (getAlterationDegrees(alteration)));
    }
    public void moveArmToScoreSpecimenPosition(double alteration){
        setArmPosition(ARM_SCORE_SPECIMEN + (getAlterationDegrees(alteration)));
    }
    public void moveArmToAttachHangingHookPosition(double alteration){
        setArmPosition(ARM_ATTACH_HANGING_HOOK + (getAlterationDegrees(alteration)));
    }
    public void moveArmToWinchRobotPosition(double alteration){
        setArmPosition(ARM_WINCH_ROBOT + (getAlterationDegrees(alteration)));
    }
    private void setArmPosition(double position) {
        armMotor.setTargetPosition((int)(position));
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
