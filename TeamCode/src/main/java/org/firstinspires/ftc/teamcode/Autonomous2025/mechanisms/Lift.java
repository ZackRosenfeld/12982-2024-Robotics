package org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    DcMotorEx lin1 = null;
    DcMotorEx lin2 = null;

    public final double COUNTS_PER_REV_LIFT = 2150.8;
    public final double INCHES_PER_REV_LIFT = 4.72441;
    public final double COUNTS_PER_INCH_LIFT = COUNTS_PER_REV_LIFT / INCHES_PER_REV_LIFT;

    // Constructor
    // Configures motors
    public Lift(HardwareMap hardwareMap) {
        lin1 = hardwareMap.get(DcMotorEx.class, "linearSlide1");
        lin2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");

        lin1.setDirection(DcMotorSimple.Direction.FORWARD);
        lin2.setDirection(DcMotorSimple.Direction.FORWARD);

        lin1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lin2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lin1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lin1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
