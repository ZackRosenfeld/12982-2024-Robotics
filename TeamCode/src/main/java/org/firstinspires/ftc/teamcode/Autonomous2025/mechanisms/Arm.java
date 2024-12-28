package org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    // Hardware
    public DcMotorEx armMotor = null; // DcMotorEx used in case velocity control is needed (it probably wont be)

    // constants used for movement
    public final double COUNTS_PER_REV_ARM = 21124.4;
    public final double COUNTS_PER_DEGREE_ARM = COUNTS_PER_REV_ARM / 360.0;

    // constructor
    // Configures motor
    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
