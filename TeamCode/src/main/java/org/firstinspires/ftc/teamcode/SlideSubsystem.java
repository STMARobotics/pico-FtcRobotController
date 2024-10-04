package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideSubsystem {

    private DcMotor slideMoter;
    public static final String SLIDE_MOTOR="slide-motor";

    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;

    double liftposition = LIFT_COLLAPSED;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public SlideSubsystem (HardwareMap hm, Telemetry telemetry){
        this.hardwareMap = hm;
        this.telemetry = telemetry;

        slideMoter = hardwareMap.get(DcMotor.class, SLIDE_MOTOR);

        slideMoter.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMoter.setTargetPosition(0);
        slideMoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMoter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void changePosition(int positionDelta){
        this.liftposition = this.liftposition + positionDelta;
        setPosition(this.liftposition);
    }

    private void setPosition(double liftposition) {
        slideMoter.setTargetPosition((int) (liftposition));

        ((DcMotorEx) slideMoter).setVelocity(2100);
        slideMoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
