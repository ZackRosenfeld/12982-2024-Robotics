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
public class Lift {
    DcMotorEx lin1 = null;
    DcMotorEx lin2 = null;

    //movement constants
    public static double LIFT_POWER = 0.65; // power sent to lift motors
    private static final double COUNTS_PER_REV_LIFT = 2150.8/4;
    private static final double INCHES_PER_REV_LIFT = 4.72441;
    private static final double COUNTS_PER_INCH_LIFT = COUNTS_PER_REV_LIFT / INCHES_PER_REV_LIFT;

    // positions
    public static double TEST_POSITION = 0;
    public static double DOWN_POS = 0;
    public static double HIGH_BAR_POS = 26 * COUNTS_PER_INCH_LIFT;
    public static double HIGH_BAR_SCORING_POS = 15 * COUNTS_PER_INCH_LIFT;

    // maximum time in seconds before lift move is ended in case a very low voltage is being sent to the motors
    public static double TIMEOUT_TIME = 6;

    ElapsedTime timer = new ElapsedTime();



    // Constructor
    // Configures motors
    public Lift(HardwareMap hardwareMap) {
        lin1 = hardwareMap.get(DcMotorEx.class, "linearSlide1");
        lin2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");

        lin1.setDirection(DcMotorEx.Direction.FORWARD);
        lin2.setDirection(DcMotorEx.Direction.REVERSE);

        lin1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lin2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lin1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lin1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /** Utilites **/

    // returns true if either lift motor is powered
    // otherwise returns false
    public boolean isBusy() {
        return lin1.isBusy() || lin2.isBusy();
    }

    // Position based utilities
    public double getCurrentPositionLin1Ticks() {
        return lin1.getCurrentPosition();
    }

    public double getCurrentPositionLin1Inches() {
        return lin1.getCurrentPosition() / COUNTS_PER_INCH_LIFT;
    }

    public double getCurrentPositionLin2Ticks() {
        return lin2.getCurrentPosition();
    }

    public double getCurrentPositionLin2Inches() {
        return lin2.getCurrentPosition() / COUNTS_PER_INCH_LIFT;
    }


    // Actions
    public class TestPosition implements Action {

        private boolean initialized = false; // stores whether or not motor has been powered already

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                lin1.setTargetPosition((int) (TEST_POSITION * COUNTS_PER_INCH_LIFT));
                lin2.setTargetPosition((int) (TEST_POSITION * COUNTS_PER_INCH_LIFT));

                lin1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                lin2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                lin1.setPower(LIFT_POWER);
                lin2.setPower(LIFT_POWER);

                timer.reset();

                initialized = true;
            }

            // checks lift's current position
            telemetryPacket.put("liftPos: ", lin1.getCurrentPosition());
            if ((lin1.isBusy() || lin2.isBusy()) && timer.seconds() < TIMEOUT_TIME) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                lin1.setPower(0);
                lin2.setPower(0);

                return false;
            }
        }
    }

    // function ofr function calls
    public Action testPosition() {
        return new TestPosition();
    }

    public class AboveHighBar implements Action {

        private boolean initialized = false; // stores whether or not motor has been powered already

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                lin1.setTargetPosition((int) HIGH_BAR_POS);
                lin2.setTargetPosition((int) HIGH_BAR_POS);

                lin1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                lin2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                lin1.setPower(LIFT_POWER);
                lin2.setPower(LIFT_POWER);

                timer.reset();

                initialized = true;
            }

            // checks lift's current position
            telemetryPacket.put("liftPos: ", lin1.getCurrentPosition());
            if ((lin1.isBusy() || lin2.isBusy()) && timer.seconds() < TIMEOUT_TIME) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                lin1.setPower(0);
                lin2.setPower(0);

                return false;
            }
        }
    }

    // function for function calls
    public Action aboveHighBar() {
        return new AboveHighBar();
    }

    public class ScoringHighBar implements Action {

        private boolean initialized = false; // stores whether or not motor has been powered already

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                lin1.setTargetPosition((int) HIGH_BAR_SCORING_POS);
                lin2.setTargetPosition((int) HIGH_BAR_SCORING_POS);

                lin1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                lin2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                lin1.setPower(LIFT_POWER);
                lin2.setPower(LIFT_POWER);

                timer.reset();

                initialized = true;
            }

            // checks lift's current position
            telemetryPacket.put("liftPos: ", lin1.getCurrentPosition());
            if ((lin1.isBusy() || lin2.isBusy()) && timer.seconds() < TIMEOUT_TIME) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                lin1.setPower(0);
                lin2.setPower(0);

                return false;
            }
        }
    }

    public Action scoringHighBar() {
        return new ScoringHighBar();
    }

    public class Down implements Action {

        private boolean initialized = false; // stores whether or not motor has been powered already

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                lin1.setTargetPosition((int) DOWN_POS);
                lin2.setTargetPosition((int) DOWN_POS);

                lin1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                lin2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                lin1.setPower(LIFT_POWER);
                lin2.setPower(LIFT_POWER);

                timer.reset();

                initialized = true;
            }

            // checks lift's current position
            telemetryPacket.put("liftPos: ", lin1.getCurrentPosition());
            if ((lin1.isBusy() || lin2.isBusy()) && timer.seconds() < TIMEOUT_TIME) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                lin1.setPower(0);
                lin2.setPower(0);

                return false;
            }
        }
    }

    public Action down() {
        return new Down();
    }
}