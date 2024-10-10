package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideSubsystem {

    private DcMotor slideMotor;
    public static final String SLIDE_MOTOR="slide-motor";

    public static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    public static final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    public static final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    public static final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_COLLAPSED;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public SlideSubsystem (HardwareMap hm, Telemetry telemetry){
        this.hardwareMap = hm;
        this.telemetry = telemetry;

        slideMotor = hardwareMap.get(DcMotor.class, SLIDE_MOTOR);

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void changePosition(int positionDelta){
        this.liftPosition = this.liftPosition + positionDelta;
        setPosition(this.liftPosition);
    }

    public void setPosition(double liftPosition) {
        slideMotor.setTargetPosition((int) (liftPosition));

        ((DcMotorEx) slideMotor).setVelocity(2100);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
