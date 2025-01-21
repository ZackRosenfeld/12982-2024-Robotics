package org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class Arm {

    // Hardware
    public DcMotorEx armMotor = null; // DcMotorEx used in case velocity control is needed (it probably wont be)

    // constants used for movement
    private static final double COUNTS_PER_REV_ARM = 5281.1;
    private static final double COUNTS_PER_DEGREE_ARM = COUNTS_PER_REV_ARM / 360.0;
    public static double ARM_POWER = 1;

    public static double TEST_POSITION = 0;

    public static double TIMEOUT_TIME = 6;
    ElapsedTime timer = new ElapsedTime();



    // constructor
    // Configures motor
    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Utilities **/

    // returns true if armMotor is being powered
    // Otherwise, returns false
    public boolean isBusy() {
        return armMotor.isBusy();
    }

    // returns current position of armMotor in ticks
    public double getCurrentPositionTicks() {
        return armMotor.getCurrentPosition();
    }

    public double getCurrentPositionDegrees() {
        return armMotor.getCurrentPosition() / COUNTS_PER_DEGREE_ARM;
    }

    // generic run to position function
    // all specific positions should be a subclass of this
    public class RunToPosition implements Action {

        private boolean initialized = false; // stores whether or not motor has been powered already
        public double POSITION = 0;

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                armMotor.setTargetPosition((int) (POSITION * COUNTS_PER_DEGREE_ARM));

                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                armMotor.setPower(ARM_POWER);

                timer.reset();

                initialized = true;
            }

            if (armMotor.isBusy() && timer.seconds() < TIMEOUT_TIME) {
                telemetryPacket.put("Arm Position (Ticks): ", armMotor.getCurrentPosition());
                telemetryPacket.put("Arm Position (Degrees): ", armMotor.getCurrentPosition() / COUNTS_PER_DEGREE_ARM);

                return true; // returning true will cause action to rerun
            }
            else {
                armMotor.setPower(0);

                return false;
            }
        }

    }

    /** specific positions **/

    public class TestPosition extends RunToPosition {
        public TestPosition() {
            POSITION = TEST_POSITION;
        }
    }

    public Action testPosition() {
        return new TestPosition();
    }

    public class StartArm extends RunToPosition {
        public StartArm() {
            POSITION = 0;
        }
    }

    public Action startArm() {
        return new StartArm();
    }

}
